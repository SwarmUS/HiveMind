#include "interloc/InterlocStateHandler.h"
#include "states/InterlocStateContainer.h"

InterlocStateHandler::InterlocStateHandler() : m_state(InterlocStateContainer::getExampleState()) {}

void InterlocStateHandler::setState(IInterlocState& state) { m_state = state; }

void InterlocStateHandler::process() { m_state.process(*this); }

TwoWayRanging& InterlocStateHandler::getTWR() { return m_twr; }
