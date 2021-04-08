#include "states/DefaultState.h"
#include "interloc/InterlocStateHandler.h"
#include <Task.h>

DefaultState::DefaultState(ILogger& logger,
                           InterlocManager& interlocManager,
                           DecawaveArray& decawaves) :
    AbstractInterlocState(logger, interlocManager, decawaves) {}

void DefaultState::process(InterlocStateHandler& context) {
    (void)context;
    // Set next state here
    // context.setState(STATE);
    Task::delay(100);
}
