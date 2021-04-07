#include "interloc/InterlocStateHandler.h"
#include "states/InterlocStateContainer.h"

InterlocStateHandler::InterlocStateHandler() : m_state(InterlocStateContainer::getExampleState()) {}

void InterlocStateHandler::setState(IInterlocState& state) {
    m_state.exit(*this);
    m_state = state;
    m_state.enter(*this);
}

void InterlocStateHandler::process() { m_state.process(*this); }
