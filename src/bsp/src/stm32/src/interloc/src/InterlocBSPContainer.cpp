#include "interloc/InterlocBSPContainer.h"
#include <logger/LoggerContainer.h>

InterlocManager& InterlocBSPContainer::getInterlocManager() {
    static InterlocManager s_manager(LoggerContainer::getLogger(), getStateHandler(),
                                     getDecawaves());

    return s_manager;
}

InterlocStateHandler& InterlocBSPContainer::getStateHandler() {
    static InterlocStateHandler s_stateHandler(getTimeManager());

    return s_stateHandler;
}

DecawaveArray& InterlocBSPContainer::getDecawaves() {
    static DecawaveArray s_decawaves;

    return s_decawaves;
}

InterlocTimeManager& InterlocBSPContainer::getTimeManager() {
    static InterlocTimeManager s_timeManager;

    return s_timeManager;
}
