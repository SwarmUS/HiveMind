#include "interloc/Interloc.h"
#include <cmath>

#define ALPHA 0.85F

Interloc::Interloc(ILogger& logger,
                   IInterlocManager& interlocManager,
                   IInterlocMessageHandler& messageHandler,
                   ICircularQueue<uint16_t>& positionUpdateOutputQueue,
                   INotificationQueue<InterlocUpdate>& positionUpdateInputQueue) :
    m_logger(logger),
    m_interlocManager(interlocManager),
    m_messageHandler(messageHandler),
    m_positionsTable(),
    m_positionUpdateOutputQueue(positionUpdateOutputQueue),
    m_positionUpdateInputQueue(positionUpdateInputQueue),
    m_updateHistoryIdx(0) {}

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
        positionToUpdate.m_distance =
            filterValue(positionToUpdate.m_distance, update.m_distance.value(), ALPHA);
    }

    if (update.m_angleOfArrival) {
        // Angles are averaged using a circular mean as to alleviate the effects of discontinuities
        // around +/- 180 degrees.
        // https://en.wikipedia.org/wiki/Circular_mean
        float newAngle = update.m_angleOfArrival.value() / 180.0F * (float)M_PI;

        positionToUpdate.m_angleRealMean =
            filterValue(positionToUpdate.m_angleRealMean, std::cos(newAngle), ALPHA);
        positionToUpdate.m_angleImaginaryMean =
            filterValue(positionToUpdate.m_angleImaginaryMean, std::sin(newAngle), ALPHA);

        positionToUpdate.m_angle =
            std::atan2(positionToUpdate.m_angleImaginaryMean, positionToUpdate.m_angleRealMean) *
            180 / (float)M_PI;
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

        if (m_updateHistoryIdx == InterlocDumpDTO::MAX_UPDATES_SIZE) {
            dumpUpdatesHistory();
        }
    }
}

void Interloc::dumpUpdatesHistory() {
    if (m_messageHandler.getDumpEnabled()) {
        if (!m_messageHandler.sendInterlocDump(m_updatesHistory.data(), m_updateHistoryIdx)) {
            m_logger.log(LogLevel::Warn, "Could not send interloc updates dump back to host");
        }
    }

    m_updateHistoryIdx = 0;
}

float Interloc::filterValue(float oldValue, float newValue, float alpha) {
    // Apply an exponential moving average filter so we don't have to accumulate old samples
    // https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
    return (alpha * newValue) + (1 - alpha) * oldValue;
}
