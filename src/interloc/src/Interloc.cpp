#include "interloc/Interloc.h"

Interloc::Interloc(ILogger& logger,
                   IInterlocManager& interlocManager,
                   ICircularQueue<uint16_t>& positionUpdateQueue) :
    m_logger(logger),
    m_interlocManager(interlocManager),
    m_positionsTable(),
    m_positionUpdateQueue(positionUpdateQueue) {
    m_interlocManager.setPositionUpdateCallback(onPositionUpdateStaticCallback, this);
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

void Interloc::onPositionUpdateCallback(InterlocUpdate positionUpdate) {
    // TODO: If we implement some long running operations here (eg filtering), should change this
    // for a queue of updates, that is processed by another thread
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

    m_positionUpdateQueue.push(positionUpdate.m_robotId);
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

    if (update.m_relativeOrientation) {
        positionToUpdate.m_relativeOrientation = update.m_relativeOrientation.value();
    }

    if (update.m_angleOfArrival) {
        positionToUpdate.m_angle = update.m_angleOfArrival.value();
    }

    if (update.m_isInLineOfSight) {
        positionToUpdate.m_isInLineOfSight = update.m_isInLineOfSight.value();
    }
}

const PositionsTable& Interloc::getPositionsTable() const { return m_positionsTable; }

void Interloc::onPositionUpdateStaticCallback(void* context, InterlocUpdate update) {
    static_cast<Interloc*>(context)->onPositionUpdateCallback(update);
}
