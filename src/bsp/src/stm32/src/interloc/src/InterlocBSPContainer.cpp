#include "interloc/InterlocBSPContainer.h"
#include "interloc/InterlocManager.h"
#include <application-interface/ApplicationInterfaceContainer.h>
#include <bsp/BSPContainer.h>
#include <interloc/InterlocContainer.h>
#include <logger/LoggerContainer.h>

InterlocManager& InterlocBSPContainer::getInterlocManager() {
    static InterlocManager s_manager(LoggerContainer::getLogger(), getStateHandler(),
                                     getDecawaves(),
                                     InterlocContainer::getInterlocUpdateInputQueue(),
                                     ApplicationInterfaceContainer::getButton0CallbackRegister());

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
    static InterlocTimeManager s_timeManager(BSPContainer::getBSP());

    return s_timeManager;
}

AngleCalculator& InterlocBSPContainer::getAngleCalculator() {
    static AngleCalculator s_angleCalculator;

    return s_angleCalculator;
}
