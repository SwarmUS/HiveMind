#include "interloc/InterlocStateHandler.h"

InterlocStateHandler::InterlocStateHandler() :
    m_currentState(&InterlocStateContainer::getState(InterlocStates::DEFAULT)) {}

void InterlocStateHandler::setState(InterlocStates state) {
    m_currentState = &InterlocStateContainer::getState(state);
}

void InterlocStateHandler::process() { m_currentState->process(*this); }

TwoWayRanging& InterlocStateHandler::getTWR() { return m_twr; }
