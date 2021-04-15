#include "interloc/InterlocTimeManager.h"
#include <interloc/Decawave.h>

InterlocTimeManager::InterlocTimeManager(IBSP& bsp) : m_bsp(bsp), m_numSlots(1), m_slotId(1) {
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
    uint64_t pollRxToResponseTxOffset =
        POLL_AIR_TIME_WITH_PREAMBLE_US + POLL_TO_FIRST_RESPONSE_GUARD_US +
        (m_slotId - 1) * (RESPONSE_AIR_TIME_WITH_PREAMBLE_US + RESPONSE_RX_TO_RESPONSE_TX_GUARD_US);

    uint64_t pollTxToFinalTxOffset =
        POLL_AIR_TIME_WITH_PREAMBLE_US + POLL_TO_FIRST_RESPONSE_GUARD_US +
        m_numSlots * (RESPONSE_AIR_TIME_WITH_PREAMBLE_US + RESPONSE_RX_TO_RESPONSE_TX_GUARD_US);

    uint64_t pollRxToFinalRxOffset = pollTxToFinalTxOffset - RX_BEFORE_TX_GUARD_US;

    uint64_t slotToSlotOffset =
        pollTxToFinalTxOffset + FINAL_AIR_TIME_WITH_PREAMBLE_US + FINAL_TO_POLL_GUARD_US;

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

uint64_t InterlocTimeManager::getPollTimeout() {
    return POLL_AIR_TIME_WITH_PREAMBLE_US + TIMEOUT_GUARD_US;
}

uint64_t InterlocTimeManager::getResponseTimeout() {
    return RESPONSE_AIR_TIME_WITH_PREAMBLE_US + TIMEOUT_GUARD_US;
}

uint64_t InterlocTimeManager::getFinalTimeout() {
    return FINAL_AIR_TIME_WITH_PREAMBLE_US + TIMEOUT_GUARD_US;
}

uint32_t InterlocTimeManager::getSyncTimeoutUs() {
    uint32_t slotToSlotOffsetUs = m_slotToSlotOffsetDTU / UUS_TO_DWT_TIME;
    return slotToSlotOffsetUs + m_bsp.generateRandomNumber() % slotToSlotOffsetUs;
}

uint64_t InterlocTimeManager::getPollTxTs(uint64_t lastSlotStartTs) const {
    return lastSlotStartTs + m_slotToSlotOffsetDTU;
}

uint64_t InterlocTimeManager::getPollRxStartTs(uint64_t lastSlotStartTs) const {
    return lastSlotStartTs + m_slotToSlotOffsetDTU - RX_BEFORE_TX_GUARD_US * UUS_TO_DWT_TIME;
}
