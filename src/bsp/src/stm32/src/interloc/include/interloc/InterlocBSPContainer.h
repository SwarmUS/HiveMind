#ifndef HIVE_MIND_INTERLOCBSPCONTAINER_H
#define HIVE_MIND_INTERLOCBSPCONTAINER_H

#include "DecawaveArray.h"
#include "InterlocManager.h"
#include "InterlocStateHandler.h"
#include "InterlocTimeManager.h"

namespace InterlocBSPContainer {

    InterlocManager& getInterlocManager();
    InterlocStateHandler& getStateHandler();
    InterlocTimeManager& getTimeManager();

    DecawaveArray& getDecawaves();
} // namespace InterlocBSPContainer

#endif // HIVE_MIND_INTERLOCBSPCONTAINER_H
