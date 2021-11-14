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
#ifdef STM32F4
    static BspInterlocRawAngleData s_rawAngleData;
#elif STM32H7
    __attribute__((section(".cmbss"))) static BspInterlocRawAngleData s_rawAngleData;
#endif

    static InterlocStateHandler s_stateHandler(getTimeManager(), s_rawAngleData);

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
    __attribute__((section(".cmbss"))) static AngleCalculator s_angleCalculator(
        LoggerContainer::getLogger());

    return s_angleCalculator;
}
