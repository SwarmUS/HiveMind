#include "interloc/InterlocTimeManager.h"
#include <interloc/Decawave.h>

InterlocTimeManager::InterlocTimeManager() { updateTimings(); }

void InterlocTimeManager::updateTimings() {
    uint64_t pollRxToResponseTxOffset =
        POLL_AIR_TIME_US + POLL_TO_FIRST_RESPONSE_GUARD_US +
        (MY_SLOT - 1) * (RESPONSE_AIR_TIME_US + RESPONSE_RX_TO_RESPONSE_TX_GUARD_US);

    uint64_t pollTxToFinalTxOffset =
        POLL_AIR_TIME_US + POLL_TO_FIRST_RESPONSE_GUARD_US +
        NUM_SLOTS * (RESPONSE_AIR_TIME_US + RESPONSE_RX_TO_RESPONSE_TX_GUARD_US);

    uint64_t pollRxToFinalRxOffset = pollTxToFinalTxOffset - RX_BEFORE_TX_GUARD_US;

    uint64_t slotToSlotOffset = pollTxToFinalTxOffset + FINAL_AIR_TIME_US + FINAL_TO_POLL_GUARD_US;

    m_pollRxToResponseTxOffsetDTU = pollRxToResponseTxOffset * UUS_TO_DWT_TIME;
    m_pollTxToFinalTxOffsetDTU = pollTxToFinalTxOffset * UUS_TO_DWT_TIME;
    m_pollRxToFinalRxOffsetDTU = pollRxToFinalRxOffset * UUS_TO_DWT_TIME;
    m_slotToSlotOffsetDTU = slotToSlotOffset * UUS_TO_DWT_TIME;
}

uint64_t InterlocTimeManager::getFinalTxTs(uint64_t pollTxTs) const {
    return pollTxTs + m_pollTxToFinalTxOffsetDTU;
}

uint64_t InterlocTimeManager::getResponseTxTs(uint64_t pollRxTs) const {
    return pollRxTs + m_pollRxToResponseTxOffsetDTU;
}

uint64_t InterlocTimeManager::getFinalRxStartTs(uint64_t pollRxTs) const {
    return pollRxTs + m_pollRxToFinalRxOffsetDTU;
}

uint64_t InterlocTimeManager::getPollTimeout() { return POLL_AIR_TIME_US + TIMEOUT_GUARD_US; }
uint64_t InterlocTimeManager::getResponseTimeout() {
    return RESPONSE_AIR_TIME_US + TIMEOUT_GUARD_US;
}
uint64_t InterlocTimeManager::getFinalTimeout() { return FINAL_AIR_TIME_US + TIMEOUT_GUARD_US; }
