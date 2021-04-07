#include "states/ExampleState.h"
#include "interloc/InterlocStateHandler.h"

void ExampleState::enter(InterlocStateHandler& context) {
    // On entry
}

void ExampleState::process(InterlocStateHandler& context) {
    // Set next state here
    // context.setState(STATE);
}

void ExampleState::exit(InterlocStateHandler& context) {}
