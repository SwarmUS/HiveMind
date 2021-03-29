#ifndef __INTERLOC_H__
#define __INTERLOC_H__

#include "IInterloc.h"
#include <bsp/IInterlocManager.h>
#include <logger/ILogger.h>

class Interloc : public IInterloc {
  public:
    Interloc(ILogger& logger, IInterlocManager& interlocManager);

    std::optional<RelativePosition> getRobotPosition(uint16_t robotId) override;
    bool isLineOfSight(uint16_t robotId) override;
    const PositionsTable& getPositionsTable() override;

  private:
    void onPositionUpdateCallback(InterlocUpdate positionUpdate);
    std::optional<uint8_t> getRobotArrayIndex(uint16_t robotId);
    static void updateRobotPosition(RelativePosition& positionToUpdate, InterlocUpdate update);

    ILogger& m_logger;
    IInterlocManager& m_interlocManager;

    PositionsTable m_positionsTable;
};

#endif //__INTERLOC_H__
