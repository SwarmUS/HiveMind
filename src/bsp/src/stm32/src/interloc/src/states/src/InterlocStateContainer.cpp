#include "states/InterlocStateContainer.h"
#include "states/ExampleState.h"
#include <states/SendFinalState.h>
#include <states/SendPollState.h>
#include <states/SendResponseState.h>
#include <states/WaitPollState.h>

IInterlocState& InterlocStateContainer::getExampleState() {
    static ExampleState s_state;

    return s_state;
}

IInterlocState& InterlocStateContainer::getSendPollState() {
    static SendPollState s_state;

    return s_state;
}

IInterlocState& InterlocStateContainer::getSendFinalState() {
    static SendFinalState s_state;

    return s_state;
}

IInterlocState& InterlocStateContainer::getWaitPollState() {
    static WaitPollState s_state;

    return s_state;
}

IInterlocState& InterlocStateContainer::getSendResponseState() {
    static SendResponseState s_state;

    return s_state;
}