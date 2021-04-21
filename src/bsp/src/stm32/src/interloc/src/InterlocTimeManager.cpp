#include "interloc/InterlocTimeManager.h"
#include <interloc/Decawave.h>

InterlocTimeManager::InterlocTimeManager(IBSP& bsp) : m_bsp(bsp), m_numSlots(0), m_slotId(0) {
    updateTimings();
}

void InterlocTimeManager::setNumSlots(uint16_t numSlots) {
    m_numSlots = numSlots;
    updateTimings();
}

void InterlocTimeManager::setSlodId(uint16_t slotId) {
    m_slotId = slotId;
    updateTimings();
}

void InterlocTimeManager::updateTimings() {
    volatile uint64_t pollRxToResponseTxOffset =
        POLL_AIR_TIME_WITH_PREAMBLE_US + POLL_TO_FIRST_RESPONSE_GUARD_US +
        (uint64_t)(m_slotId - 1) *
            (uint64_t)(RESPONSE_AIR_TIME_WITH_PREAMBLE_US + RESPONSE_TO_RESPONSE_GUARD_US);

    volatile uint64_t pollTxToFinalTxOffset =
        POLL_AIR_TIME_WITH_PREAMBLE_US + POLL_TO_FIRST_RESPONSE_GUARD_US +
        m_numSlots * (RESPONSE_AIR_TIME_WITH_PREAMBLE_US + RESPONSE_TO_RESPONSE_GUARD_US);

    volatile uint64_t pollRxToFinalRxOffset = pollTxToFinalTxOffset - RX_BEFORE_TX_GUARD_US;

    volatile uint64_t slotToSlotOffsetUs =
        pollTxToFinalTxOffset + FINAL_AIR_TIME_WITH_PREAMBLE_US + FINAL_TO_POLL_GUARD_US;

    for (unsigned int slotIdx = 1; slotIdx <= m_numSlots; slotIdx++) {
        m_RespRxStartOffset[slotIdx - 1] =
            UUS_TO_DWT_TIME *
            ((slotIdx - 1) * (RESPONSE_AIR_TIME_WITH_PREAMBLE_US + RESPONSE_TO_RESPONSE_GUARD_US) +
             POLL_TO_FIRST_RESPONSE_GUARD_US - RX_BEFORE_TX_GUARD_US);
    }

    m_pollRxToResponseTxOffsetDTU = pollRxToResponseTxOffset * UUS_TO_DWT_TIME;
    m_pollTxToFinalTxOffsetDTU = pollTxToFinalTxOffset * UUS_TO_DWT_TIME;
    m_pollRxToFinalRxOffsetDTU = pollRxToFinalRxOffset * UUS_TO_DWT_TIME;
    m_slotToSlotOffsetDTU = slotToSlotOffsetUs * UUS_TO_DWT_TIME;
}

uint64_t InterlocTimeManager::getFinalTxTs(uint64_t pollTxTs) const {
    return (pollTxTs + m_pollTxToFinalTxOffsetDTU) % UINT40_MAX;
}

uint64_t InterlocTimeManager::getResponseTxTs(uint64_t pollRxTs) const {
    return (pollRxTs + m_pollRxToResponseTxOffsetDTU) % UINT40_MAX;
}

uint64_t InterlocTimeManager::getFinalRxStartTs(uint64_t pollRxTs) const {
    return (pollRxTs + m_pollRxToFinalRxOffsetDTU) % UINT40_MAX;
}

uint64_t InterlocTimeManager::getPollTimeout() const {
    // TODO: Change
    return 2 * m_slotToSlotOffsetDTU / UUS_TO_DWT_TIME;
}

uint64_t InterlocTimeManager::getResponseTimeout() {
    //    return m_pollTxToFinalTxOffsetDTU / UUS_TO_DWT_TIME - RESPONSE_TO_FINAL_GUARD_US;
    return RESPONSE_AIR_TIME_WITH_PREAMBLE_US + 1500;
}

uint64_t InterlocTimeManager::getFinalTimeout() { return FINAL_AIR_TIME_WITH_PREAMBLE_US + 4000; }

uint32_t InterlocTimeManager::getSyncTimeoutUs() {
    uint32_t slotToSlotOffsetUs = m_slotToSlotOffsetDTU / UUS_TO_DWT_TIME;
    return slotToSlotOffsetUs + m_bsp.generateRandomNumber() % slotToSlotOffsetUs;
}

uint64_t InterlocTimeManager::getPollTxTs(uint64_t lastSlotStartTs) const {
    return (lastSlotStartTs + m_slotToSlotOffsetDTU) % UINT40_MAX;
}

uint64_t InterlocTimeManager::getPollRxStartTs(uint64_t lastSlotStartTs) const {
    return (lastSlotStartTs + m_slotToSlotOffsetDTU - RX_BEFORE_TX_GUARD_US * UUS_TO_DWT_TIME) %
           UINT40_MAX;
}

uint64_t InterlocTimeManager::getRespRxStartTime(uint64_t pollTxTs, uint8_t slotIdx) {
    // TODO reference the time with an actual getPollTxTs
    // start to listen a little bit before the actual Response is sent.
    return pollTxTs + m_RespRxStartOffset[slotIdx];
}