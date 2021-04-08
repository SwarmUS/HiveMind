#include "interloc/InterlocBSPContainer.h"
#include <array>
#include <logger/LoggerContainer.h>

InterlocManager& InterlocBSPContainer::getInterlocManager() {
    static InterlocManager s_manager(LoggerContainer::getLogger(), getStateHandler());

    return s_manager;
}

InterlocStateHandler& InterlocBSPContainer::getStateHandler() {
    static InterlocStateHandler s_stateHandler;

    return s_stateHandler;
}
Decawave& InterlocBSPContainer::getDecawave(InterlocBSPContainer::DecawavePort port) {
    static std::array<Decawave, gc_numDecawavePorts> s_decawaves = {
        Decawave(DW_A, UWBChannel::CHANNEL_5, UWBSpeed::SPEED_6M8),
        Decawave(DW_B, UWBChannel::CHANNEL_5, UWBSpeed::SPEED_6M8)};

    return s_decawaves.at(static_cast<unsigned int>(port));
}
