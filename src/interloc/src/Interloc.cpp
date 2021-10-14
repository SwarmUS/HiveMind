#include "interloc/Interloc.h"

void Interloc::task(void* context) {
    while (true) {
        static_cast<Interloc*>(context)->process();
    }
}

Interloc::Interloc(ILogger& logger,
                   IInterlocManager& interlocManager,
                   IInterlocMessageHandler& messageHandler,
                   ICircularQueue<uint16_t>& positionUpdateQueue,
                   NotificationQueue<InterlocUpdate>& positionUpdateInputQueue) :
    m_logger(logger),
    m_interlocManager(interlocManager),
    m_messageHandler(messageHandler),
    m_positionsTable(),
    m_positionUpdateOutputQueue(positionUpdateQueue),
    m_positionUpdateInputQueue(positionUpdateInputQueue),
    m_updateHistoryIdx(0),
    m_processTask("interloc_update_handler", tskIDLE_PRIORITY + 1, task, this) {
    m_processTask.start();
}

std::optional<RelativePosition> Interloc::getRobotPosition(uint16_t robotId) const {
    std::optional<uint8_t> idx = getRobotArrayIndex(robotId);

    if (idx) {
        return m_positionsTable.m_positions[idx.value()];
    }

    return {};
}

bool Interloc::isLineOfSight(uint16_t robotId) const {
    std::optional<RelativePosition> position = getRobotPosition(robotId);

    if (position) {
        return position->m_isInLineOfSight;
    }

    return {};
}

void Interloc::processPositionUpdate(const InterlocUpdate& positionUpdate) {
    std::optional<uint8_t> idx = getRobotArrayIndex(positionUpdate.m_robotId);

    if (idx) {
        updateRobotPosition(m_positionsTable.m_positions[idx.value()], positionUpdate);
    } else {
        if (m_positionsTable.m_positionsLength == m_positionsTable.m_positions.size()) {
            m_logger.log(LogLevel::Error, "Robot positions array too small for number of robots");
            return;
        }
        updateRobotPosition(m_positionsTable.m_positions[m_positionsTable.m_positionsLength++],
                            positionUpdate);
    }

    m_positionUpdateOutputQueue.push(positionUpdate.m_robotId);
}

std::optional<uint8_t> Interloc::getRobotArrayIndex(uint16_t robotId) const {
    // TODO: migrate to a hashmap
    for (unsigned int i = 0; i < m_positionsTable.m_positions.size(); i++) {
        if (m_positionsTable.m_positions[i].m_robotId == robotId) {
            return i;
        }
    }

    return {};
}

void Interloc::updateRobotPosition(RelativePosition& positionToUpdate, InterlocUpdate update) {
    positionToUpdate.m_robotId = update.m_robotId;

    if (update.m_distance) {
        positionToUpdate.m_distance = update.m_distance.value();
    }

    if (update.m_angleOfArrival) {
        positionToUpdate.m_angle = update.m_angleOfArrival.value();
    }

    if (update.m_isInLineOfSight) {
        positionToUpdate.m_isInLineOfSight = update.m_isInLineOfSight.value();
    }
}

const PositionsTable& Interloc::getPositionsTable() const { return m_positionsTable; }

void Interloc::process() {
    if (m_positionUpdateInputQueue.isEmpty()) {
        m_positionUpdateInputQueue.wait(500);
    }

    if (auto update = m_positionUpdateInputQueue.peek()) {
        processPositionUpdate(update->get());
        m_updatesHistory[m_updateHistoryIdx] = update->get();
        m_updateHistoryIdx++;

        m_positionUpdateInputQueue.pop();

        if (m_updateHistoryIdx == InterlocDumpDTO::MAX_UPDATES_SIZE - 1) {
            dumpUpdatesHistory();
        }
    }
}

void Interloc::dumpUpdatesHistory() {
    if (m_messageHandler.getDumpEnabled()) {
        if (!m_messageHandler.sendInterlocDump(m_updatesHistory.data(), m_updateHistoryIdx + 1)) {
            m_logger.log(LogLevel::Warn, "Could not send interloc updates dump back to host");
        }
    }

    m_updateHistoryIdx = 0;
}
