#ifndef __STATETRANSITION_H__
#define __STATETRANSITION_H__

#include "InterlocStateContainer.h"

enum class InterlocEvent {
    NO_EVENT,
    TIMEOUT,
    POLL_RECVD,
    FINAL_RECVD,
    RESPONSE_RECVD,
    DISTANCE_ERROR,
    RX_ERROR,
    RX_LAST_RESP,
    RX_RESP,
    RX_RESP_GOOD,
    RX_LAST_TIMEOUT,
    RX_NO_RESP,
    NEXT_FRAME_LEADER,
    NOT_NEXT_FRAME_LEADER,
    BOOTUP,
    SUPERFRAME_FINISHED,
    GOOD_DISTANCE,
    BAD_DISTANCE,
    API_MESSAGE,
    ANGLE_RECEIVED,
    ANGLES_SENT
};

struct StateTransition {
    InterlocStates m_fromState;
    InterlocStates m_toState;
    InterlocEvent m_transitionEvent;
    uint8_t m_currentFrameId;
};

#endif //__STATETRANSITION_H__
