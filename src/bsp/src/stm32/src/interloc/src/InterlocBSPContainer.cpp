#include "interloc/InterlocBSPContainer.h"
#include <logger/LoggerContainer.h>

InterlocManager& InterlocBSPContainer::getInterlocManager() {
    static InterlocManager s_manager(LoggerContainer::getLogger(), getStateHandler());

    return s_manager;
}

InterlocStateHandler& InterlocBSPContainer::getStateHandler() {
    static InterlocStateHandler s_stateHandler;

    return s_stateHandler;
}