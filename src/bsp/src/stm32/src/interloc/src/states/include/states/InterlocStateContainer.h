#ifndef HIVE_MIND_INTERLOCSTATECONTAINER_H
#define HIVE_MIND_INTERLOCSTATECONTAINER_H

#include "AbstractInterlocState.h"

enum class InterlocStates {
    DEFAULT = 0,
    SEND_POLL,
    SEND_FINAL,
    WAIT_POLL,
    WAIT_FINAL,
    WAIT_RESPONSE,
    SEND_RESPONSE,
    NUM_STATES
};

namespace InterlocStateContainer {
    AbstractInterlocState& getState(InterlocStates state);
} // namespace InterlocStateContainer

#endif // HIVE_MIND_INTERLOCSTATECONTAINER_H
