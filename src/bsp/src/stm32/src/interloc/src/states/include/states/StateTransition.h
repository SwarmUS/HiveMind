#ifndef __STATETRANSITION_H__
#define __STATETRANSITION_H__

#include "InterlocStateContainer.h"

enum class InterlocEvent { NO_EVENT, TIMEOUT, POLL_RECVD, FINAL_RECVD, RESPONSE_RECVD };

struct StateTransition {
    InterlocStates m_fromState;
    InterlocStates m_toState;
    InterlocEvent m_transitionEvent;
};

#endif //__STATETRANSITION_H__
