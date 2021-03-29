#include "interloc/Interloc.h"
#include <optional>

Interloc::Interloc(ILogger& logger, IInterlocManager& interlocManager) :
    m_logger(logger),
    m_interlocManager(interlocManager),
    m_robotPositions({}),
    m_robotPositionsLength(0) {
    m_interlocManager.registerDataCallback(
        [this](InterlocUpdate position) { onDataCallback(InterlocUpdate(position)); });
}

std::optional<RelativePosition> Interloc::getRobotPosition(uint16_t robotId) {
    int16_t idx = getRobotArrayIndex(robotId);

    if (idx >= 0) {
        return m_robotPositions[idx];
    }

    return {};
}

void Interloc::onDataCallback(InterlocUpdate positionUpdate) {
    int16_t idx = getRobotArrayIndex(positionUpdate.m_robotId);

    if (idx >= 0) {
        updateRobotPosition(m_robotPositions[idx], positionUpdate);
    } else {
        if (m_robotPositionsLength == m_robotPositions.size()) {
            m_logger.log(LogLevel::Error, "Robot positions array too small for number of robots");
            return;
        }
        updateRobotPosition(m_robotPositions[m_robotPositionsLength++], positionUpdate);
    }
}

int16_t Interloc::getRobotArrayIndex(uint16_t robotId) {
    for (int i = 0; i < m_robotPositions.size(); i++) {
        if (m_robotPositions[i].m_robotId == robotId) {
            return i;
        }
    }

    return -1;
}

void Interloc::updateRobotPosition(RelativePosition& positionToUpdate, InterlocUpdate update) {
    positionToUpdate.m_robotId = update.m_robotId;

    if (update.m_distance) {
        positionToUpdate.m_distance = update.m_distance.value();
    }

    if (update.m_relativeOrientation) {
        positionToUpdate.m_relativeOrientation = update.m_relativeOrientation.value();
    }

    if (update.m_isInLineOfSight) {
        positionToUpdate.m_isInLineOfSight = update.m_isInLineOfSight.value();
    }
}
