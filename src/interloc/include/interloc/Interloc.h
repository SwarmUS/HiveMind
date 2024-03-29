#ifndef __INTERLOC_H__
#define __INTERLOC_H__

#include "IInterloc.h"
#include "IInterlocMessageHandler.h"
#include <BaseTask.h>
#include <ConditionVariable.h>
#include <Mutex.h>
#include <NotificationQueue.h>
#include <OSMacros.h>
#include <ThreadSafeQueue.h>
#include <bsp/IInterlocManager.h>
#include <cpp-common/CircularQueueStack.h>
#include <logger/ILogger.h>
#include <pheromones/MessageDTO.h>
#include <pheromones/interloc/InterlocDumpDTO.h>

class Interloc : public IInterloc {
  public:
    static constexpr uint8_t INTERLOC_UDPATES_QUEUE_SIZE = 5;

    Interloc(ILogger& logger,
             IInterlocManager& interlocManager,
             IInterlocMessageHandler& messageHandler,
             ICircularQueue<uint16_t>& positionUpdateOutputQueue,
             INotificationQueue<InterlocUpdate>& positionUpdateInputQueue);
    virtual ~Interloc() = default;

    std::optional<RelativePosition> getRobotPosition(uint16_t robotId) const override;
    bool isLineOfSight(uint16_t robotId) const override;
    const PositionsTable& getPositionsTable() const override;

    void process();

  private:
    std::optional<uint8_t> getRobotArrayIndex(uint16_t robotId) const;
    static void updateRobotPosition(RelativePosition& positionToUpdate, InterlocUpdate update);

    ILogger& m_logger;
    IInterlocManager& m_interlocManager;
    IInterlocMessageHandler& m_messageHandler;

    PositionsTable m_positionsTable;
    ICircularQueue<uint16_t>& m_positionUpdateOutputQueue;
    INotificationQueue<InterlocUpdate>& m_positionUpdateInputQueue;

    std::array<InterlocUpdate, InterlocDumpDTO::MAX_UPDATES_SIZE> m_updatesHistory;
    uint8_t m_updateHistoryIdx;

    void processPositionUpdate(const InterlocUpdate& positionUpdate);
    void dumpUpdatesHistory();
    static constexpr float filterValue(float oldValue, float newValue, float alpha);
};

#endif //__INTERLOC_H__
