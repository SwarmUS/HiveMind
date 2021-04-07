#include "states/InterlocStateContainer.h"
#include "states/ExampleState.h"

IInterlocState& InterlocStateContainer::getExampleState() {
    static ExampleState s_state;

    return s_state;
}