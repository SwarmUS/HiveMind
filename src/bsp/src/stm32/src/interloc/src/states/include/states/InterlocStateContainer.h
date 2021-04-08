#ifndef HIVE_MIND_INTERLOCSTATECONTAINER_H
#define HIVE_MIND_INTERLOCSTATECONTAINER_H

#include "IInterlocState.h"

namespace InterlocStateContainer {
    IInterlocState& getExampleState();
    IInterlocState& getSendPollState();
    IInterlocState& getSendFinalState();
    IInterlocState& getWaitPollState();
    IInterlocState& getSendResponseState();

} // namespace InterlocStateContainer

#endif // HIVE_MIND_INTERLOCSTATECONTAINER_H
