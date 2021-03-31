#ifndef __INTERLOC_H__
#define __INTERLOC_H__

#include "IInterloc.h"
#include "IInterlocMessageHandler.h"
#include <bsp/IInterlocManager.h>
#include <logger/ILogger.h>

class Interloc : public IInterloc {
  public:
    Interloc(ILogger& logger,
             IInterlocManager& interlocManager,
             IInterlocMessageHandler& interlocMessageHandler);
    virtual ~Interloc() = default;

    std::optional<RelativePosition> getRobotPosition(uint16_t robotId) override;
    bool isLineOfSight(uint16_t robotId) override;
    const PositionsTable& getPositionsTable() override;

  private:
    void onPositionUpdateCallback(InterlocUpdate positionUpdate);
    void onCalibrationEndedCallback(uint16_t initiatorId);
    std::optional<uint8_t> getRobotArrayIndex(uint16_t robotId);
    static void updateRobotPosition(RelativePosition& positionToUpdate, InterlocUpdate update);

    static void onPositionUpdateStaticCallback(void* context, InterlocUpdate update);
    static void onCalibrationEndedStaticCallback(void* context, uint16_t initiatorId);

    ILogger& m_logger;
    IInterlocManager& m_interlocManager;
    IInterlocMessageHandler& m_interlocMessageHandler;

    PositionsTable m_positionsTable;
};

#endif //__INTERLOC_H__
