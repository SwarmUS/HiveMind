#ifndef HIVE_MIND_INTERLOCSTATEHANDLER_H
#define HIVE_MIND_INTERLOCSTATEHANDLER_H

#include "InterlocTimeManager.h"
#include "TwoWayRanging.h"
#include <array>
#include <cpp-common/CircularQueue.h>
#include <states/AbstractInterlocState.h>
#include <states/InterlocStateContainer.h>
#include <states/StateTransition.h>

// TODO: Add to settings
#define PAN_ID 0x01
#define MAX_TRACER_TRANSITIONS 500

class InterlocStateHandler {
  public:
    InterlocStateHandler(InterlocTimeManager& timeManager);

    void setState(InterlocStates state, InterlocEvent event);
    void process();

    bool constructUWBHeader(uint16_t destinationId,
                            UWBMessages::FrameType frameType,
                            UWBMessages::FunctionCode functionCode,
                            uint8_t* buffer,
                            uint16_t bufferLength);

    TwoWayRanging& getTWR();

  private:
    InterlocTimeManager& m_timeManager;

    InterlocStates m_currentStateName;
    AbstractInterlocState* m_currentState;
    TwoWayRanging m_twr{};

    std::array<StateTransition, MAX_TRACER_TRANSITIONS> m_stateTracerData;
    CircularQueue<StateTransition> m_stateTracer;

    uint8_t m_sequenceID = 0;
};

#endif // HIVE_MIND_INTERLOCSTATEHANDLER_H
