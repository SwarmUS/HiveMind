#ifndef HIVE_MIND_INTERLOCSTATEHANDLER_H
#define HIVE_MIND_INTERLOCSTATEHANDLER_H

#include "TwoWayRanging.h"
#include <array>
#include <states/AbstractInterlocState.h>
#include <states/InterlocStateContainer.h>

// TODO: Add to settings
#define PAN_ID 0x01

class InterlocStateHandler {
  public:
    InterlocStateHandler();

    void setState(InterlocStates state);
    void process();

    bool constructUWBHeader(uint16_t destinationId,
                            UWBMessages::FrameType frameType,
                            UWBMessages::FunctionCode functionCode,
                            uint8_t* buffer,
                            uint16_t bufferLength);

    TwoWayRanging& getTWR();

  private:
    AbstractInterlocState* m_currentState;
    TwoWayRanging m_twr{};

    uint8_t m_sequenceID = 0;
};

#endif // HIVE_MIND_INTERLOCSTATEHANDLER_H
