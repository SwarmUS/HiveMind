#include "interloc/Interloc.h"
#include <optional>

Interloc::Interloc(ILogger& logger, IInterlocManager& interlocManager) :
    m_logger(logger),
    m_interlocManager(interlocManager),
    m_robotPositions({}),
    m_robotPositionsLength(0) {
    m_interlocManager.registerDataCallback(
        [this](RobotPosition position) { onDataCallback(RobotPosition(position)); });
}

std::optional<RobotPosition> Interloc::getRobotPosition(uint16_t robotId) {
    int16_t idx = getRobotArrayIndex(robotId);

    if (idx >= 0) {
        return m_robotPositions[idx];
    }

    return {};
}

void Interloc::onDataCallback(RobotPosition position) {
    int16_t idx = getRobotArrayIndex(position.m_robotId);

    if (idx >= 0) {
        m_robotPositions[idx] = position;
    } else {
        if (m_robotPositionsLength == m_robotPositions.size()) {
            m_logger.log(LogLevel::Error, "Robot positions array too small for number of robots");
            return;
        }
        m_robotPositions[m_robotPositionsLength++] = position;
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
