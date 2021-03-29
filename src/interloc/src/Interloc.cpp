#include "interloc/Interloc.h"
#include <optional>

Interloc::Interloc(ILogger& logger, IInterlocManager& interlocManager) :
    m_logger(logger), m_interlocManager(interlocManager), m_positionsTable() {
    m_interlocManager.registerPositionUpdateCallback(
        [this](InterlocUpdate position) { onPositionUpdateCallback(InterlocUpdate(position)); });
}

std::optional<RelativePosition> Interloc::getRobotPosition(uint16_t robotId) {
    int16_t idx = getRobotArrayIndex(robotId);

    if (idx >= 0) {
        return m_positionsTable.m_positions[static_cast<uint16_t>(idx)];
    }

    return {};
}

bool Interloc::isLineOfSight(uint16_t robotId) {
    int16_t idx = getRobotArrayIndex(robotId);

    if (idx >= 0) {
        return m_positionsTable.m_positions[static_cast<uint16_t>(idx)].m_isInLineOfSight;
    }

    return false;
}

void Interloc::onPositionUpdateCallback(InterlocUpdate positionUpdate) {
    // TODO: If we implement some long running operations here (eg filtering), should change this
    // for a queue of updates, that is processed by another thread
    int16_t idx = getRobotArrayIndex(positionUpdate.m_robotId);

    if (idx >= 0) {
        updateRobotPosition(m_positionsTable.m_positions[static_cast<uint16_t>(idx)],
                            positionUpdate);
    } else {
        if (m_positionsTable.m_positionsLength == m_positionsTable.m_positions.size()) {
            m_logger.log(LogLevel::Error, "Robot positions array too small for number of robots");
            return;
        }
        updateRobotPosition(m_positionsTable.m_positions[m_positionsTable.m_positionsLength++],
                            positionUpdate);
    }
}

int16_t Interloc::getRobotArrayIndex(uint16_t robotId) {
    for (unsigned int i = 0; i < m_positionsTable.m_positions.size(); i++) {
        if (m_positionsTable.m_positions[i].m_robotId == robotId) {
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

const PositionsTable& Interloc::getPositionsTable() { return m_positionsTable; }