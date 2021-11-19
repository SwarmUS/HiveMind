#ifndef HIVE_MIND_INTERLOCSTATEHANDLER_H
#define HIVE_MIND_INTERLOCSTATEHANDLER_H

#include "InterlocTimeManager.h"
#include "TwoWayRanging.h"
#include <array>
#include <bsp/BspInterlocAngleRawData.h>
#include <cpp-common/CircularQueue.h>
#include <states/AbstractInterlocState.h>
#include <states/InterlocStateContainer.h>
#include <states/StateTransition.h>

#define PAN_ID 0x01
#define MAX_TRACER_TRANSITIONS 500
#define ANGLE_NUMBER_OF_FRAMES 50

class InterlocStateHandler {
  public:
    InterlocStateHandler(InterlocTimeManager& timeManager, BspInterlocRawAngleData& angleRawData);

    void setState(InterlocStates state, InterlocEvent event);
    void process();

    bool constructUWBHeader(uint16_t destinationId,
                            UWBMessages::FrameType frameType,
                            UWBMessages::FunctionCode functionCode,
                            uint8_t* buffer,
                            uint16_t bufferLength);

    TwoWayRanging& getTWR();
    BspInterlocRawAngleData& getRawAngleData();
    InterlocTimeManager& getTimeManager() const;

    void incrementCurrentTwrFrame();

    uint8_t getSlotId() const;
    uint8_t getNumFrames() const;
    uint8_t getSuperFrameInitiator() const;
    void setSuperFrameInitiator(uint8_t initiatorId);
    uint8_t getCurrentFrameId() const;
    void setCurrentFrameId(uint8_t frameId);
    uint64_t getPreviousFrameStartTs() const;
    void setPreviousFrameStartTs(uint64_t timestamp);

    void setAngleCalibNumberOfFrames(uint32_t numberOfFrames);
    uint32_t getAngleCalibNumberOfFrames() const;

    static uint16_t getSlotIdFromBoardId(uint16_t boardId);
    static uint16_t getBoardIdFromSlotId(uint16_t slotId);

  private:
    InterlocTimeManager& m_timeManager;

    InterlocStates m_currentStateName;
    AbstractInterlocState* m_currentState;
    TwoWayRanging m_twr{};
    BspInterlocRawAngleData& m_angleRawData;

    std::array<StateTransition, MAX_TRACER_TRANSITIONS> m_stateTracerData;
    CircularQueue<StateTransition> m_stateTracer;

    uint8_t m_sequenceID = 0; // Unique ID of UWB messages
    uint16_t m_slotId;

    uint8_t m_numFrames = NUM_TWR_FRAMES;
    uint8_t m_superFrameInitiator = 0;
    uint8_t m_currentTwrFrame = 0;
    uint64_t m_previousFrameStartTs = 0;

    uint32_t m_angleCalibNumberOfFrames = ANGLE_NUMBER_OF_FRAMES;
};

#endif // HIVE_MIND_INTERLOCSTATEHANDLER_H
