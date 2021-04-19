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
    RX_LAST_TIMEOUT
};

struct StateTransition {
    InterlocStates m_fromState;
    InterlocStates m_toState;
    InterlocEvent m_transitionEvent;
};

#endif //__STATETRANSITION_H__
