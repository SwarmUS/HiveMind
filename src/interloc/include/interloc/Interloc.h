#ifndef __INTERLOC_H__
#define __INTERLOC_H__

#include "IInterloc.h"
#include <bsp/IInterlocManager.h>
#include <logger/ILogger.h>
#include <optional>

// TODO: Move to settings
#define MAX_ROBOTS_IN_SWARM 10

class Interloc : public IInterloc {
  public:
    Interloc(ILogger& logger, IInterlocManager& interlocManager);

    std::optional<RobotPosition> getRobotPosition(uint16_t robotId);

  private:
    void onDataCallback(RobotPosition position);
    int16_t getRobotArrayIndex(uint16_t robotId);

    ILogger& m_logger;
    IInterlocManager& m_interlocManager;

    std::array<RobotPosition, MAX_ROBOTS_IN_SWARM> m_robotPositions;
    uint16_t m_robotPositionsLength;
};

#endif //__INTERLOC_H__
