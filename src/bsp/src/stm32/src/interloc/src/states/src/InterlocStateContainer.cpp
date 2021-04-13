#include <interloc/InterlocBSPContainer.h>
#include <logger/LoggerContainer.h>
#include <states/DefaultState.h>
#include <states/InterlocStateContainer.h>
#include <states/SendFinalState.h>
#include <states/SendPollState.h>
#include <states/SendResponseState.h>
#include <states/WaitPollState.h>
#include <states/WaitFinalState.h>
#include <states/WaitResponseState.h>


AbstractInterlocState& InterlocStateContainer::getState(InterlocStates state) {
    static DefaultState s_defaultState(LoggerContainer::getLogger(),
                                       InterlocBSPContainer::getDecawaves());

    static SendPollState s_sendPollState(LoggerContainer::getLogger(),
                                         InterlocBSPContainer::getDecawaves());

    static SendFinalState s_sendFinalState(LoggerContainer::getLogger(),
                                           InterlocBSPContainer::getDecawaves());

    static WaitPollState s_waitPollState(LoggerContainer::getLogger(),
                                         InterlocBSPContainer::getDecawaves());

    static SendResponseState s_sendResponseState(LoggerContainer::getLogger(),
                                                 InterlocBSPContainer::getDecawaves());

    static WaitFinalState s_waitFinalState(LoggerContainer::getLogger(),
                                                 InterlocBSPContainer::getDecawaves());

    static WaitResponseState s_waitResponseState(LoggerContainer::getLogger(),
                                           InterlocBSPContainer::getDecawaves());


    switch (state) {
    case InterlocStates::DEFAULT:
        return s_defaultState;

    case InterlocStates::SEND_POLL:
        return s_sendPollState;

    case InterlocStates::SEND_FINAL:
        return s_sendFinalState;

    case InterlocStates::WAIT_POLL:
        return s_waitPollState;

    case InterlocStates::SEND_RESPONSE:
        return s_sendResponseState;

    case InterlocStates::WAIT_FINAL:
        return s_waitFinalState;

    case InterlocStates::WAIT_RESPONSE:
        return s_waitResponseState;

    default:
        return s_defaultState;
    }
}