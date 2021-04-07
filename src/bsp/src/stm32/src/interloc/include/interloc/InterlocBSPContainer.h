#ifndef HIVE_MIND_INTERLOCBSPCONTAINER_H
#define HIVE_MIND_INTERLOCBSPCONTAINER_H

#include "InterlocManager.h"
#include "InterlocStateHandler.h"

namespace InterlocBSPContainer {
    enum class DecawavePort { A = 0, B = 1 };
    constexpr uint8_t gc_numDecawavePorts = 2;

    InterlocManager& getInterlocManager();
    InterlocStateHandler& getStateHandler();
    Decawave& getDecawave(DecawavePort port);
} // namespace InterlocBSPContainer

#endif // HIVE_MIND_INTERLOCBSPCONTAINER_H
