#ifndef __INTERLOC_H__
#define __INTERLOC_H__

#include "IInterloc.h"
#include "RelativePosition.h"
#include <bsp/IInterlocManager.h>
#include <logger/ILogger.h>
#include <optional>

#define MAX_ROBOTS_IN_SWARM 10

struct PositionsTable {
    std::array<RelativePosition, MAX_ROBOTS_IN_SWARM> m_positions;
    uint16_t m_positionsLength;
};

class Interloc : public IInterloc {
  public:
    Interloc(ILogger& logger, IInterlocManager& interlocManager);

    std::optional<RelativePosition> getRobotPosition(uint16_t robotId);
    bool isLineOfSight(uint16_t robotId);
    const PositionsTable& getPositionsTable();

  private:
    void onDataCallback(InterlocUpdate positionUpdate);
    int16_t getRobotArrayIndex(uint16_t robotId);
    static void updateRobotPosition(RelativePosition& positionToUpdate, InterlocUpdate update);

    ILogger& m_logger;
    IInterlocManager& m_interlocManager;

    PositionsTable m_positionsTable;
};

#endif //__INTERLOC_H__
