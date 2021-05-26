#include "interloc/InterlocTimeManager.h"
#include "interloc/InterlocStateHandler.h"
#include <interloc/Decawave.h>
#include <interloc/UWBMessages.h>

InterlocTimeManager::InterlocTimeManager(IBSP& bsp) :
    m_pollAirTimeWithPreamble(0),
    m_responseAirTimeWithPreamble(0),
    m_finalAirTimeWithPreamble(0),
    m_pollToFirstResponseGuardUs(0),
    m_bsp(bsp),
    m_numSlots(5),
    m_slotId(2)

{
    m_pollAirTimeWithPreamble = computeAirTimeWithPreambleUs(sizeof(UWBMessages::TWRPoll) << 3);
    m_responseAirTimeWithPreamble =
        computeAirTimeWithPreambleUs(sizeof(UWBMessages::TWRResponse) << 3);
    m_finalAirTimeWithPreamble = computeAirTimeWithPreambleUs(sizeof(UWBMessages::TWRFinal) << 3);
    m_pollToFirstResponseGuardUs = getReadWriteSPITimeUs(sizeof(UWBMessages::TWRPoll)) +
                                   POLL_PROCESSING_GUARD + getPreambleAirTimeUs();

    uint64_t startOfFrameTs = 0;

    computeResponseRxTs_new(startOfFrameTs);

    volatile uint64_t responseTxTs[5];
    for (unsigned int i = 1; i < 6; i++) {
        m_slotId = i;
        responseTxTs[i - 1] = getResponseTxTs_new(startOfFrameTs);
    }
    m_slotId = 2;
    volatile uint64_t finalRxTs = getFinalRxTs_new(startOfFrameTs);
    volatile uint64_t finalTxTs = getFinalTxTs_new(startOfFrameTs);
    volatile uint16_t pollTO = getTimeoutUs_new(m_pollAirTimeWithPreamble);
    volatile uint16_t responseTO = getTimeoutUs_new(m_responseAirTimeWithPreamble);
    volatile uint16_t finalTO = getTimeoutUs_new(m_finalAirTimeWithPreamble);
    volatile uint64_t newPollTs = getPollRxStartTs_new(startOfFrameTs);

    responseTxTs[0]++;
    finalRxTs++;
    finalTxTs++;
    m_responseRxTs_new[9] = 0;
    pollTO++;
    responseTO++;
    finalTO++;
    newPollTs++;

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
        m_respRxStartOffset[slotIdx - 1] =
            UUS_TO_DWT_TIME *
            ((slotIdx - 1) * (RESPONSE_AIR_TIME_WITH_PREAMBLE_US + RESPONSE_TO_RESPONSE_GUARD_US) +
             POLL_TO_FIRST_RESPONSE_GUARD_US - RX_BEFORE_TX_GUARD_US);
    }

    m_pollRxToResponseTxOffsetDTU = pollRxToResponseTxOffset * UUS_TO_DWT_TIME;
    m_pollTxToFinalTxOffsetDTU = pollTxToFinalTxOffset * UUS_TO_DWT_TIME;
    m_pollRxToFinalRxOffsetDTU = pollRxToFinalRxOffset * UUS_TO_DWT_TIME;
    m_slotToSlotOffsetDTU = slotToSlotOffsetUs * UUS_TO_DWT_TIME;
    // compute constant values air time with their preamble
    m_pollAirTimeWithPreamble = computeAirTimeWithPreambleUs(sizeof(UWBMessages::TWRPoll) << 3);
    m_responseAirTimeWithPreamble =
        computeAirTimeWithPreambleUs(sizeof(UWBMessages::TWRResponse) << 3);
    m_finalAirTimeWithPreamble = computeAirTimeWithPreambleUs(sizeof(UWBMessages::TWRFinal) << 3);
    m_pollToFirstResponseGuardUs = getReadWriteSPITimeUs(sizeof(UWBMessages::TWRPoll)) +
                                   POLL_PROCESSING_GUARD + getPreambleAirTimeUs();
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
    return pollTxTs + m_respRxStartOffset[slotIdx];
}
/*------------------------------- NEW FILE -------------------------------*/

constexpr float getShrSymbolDurationUs() {
    switch (PRF_SPEED) {
    case 16:
        return 993.59 / 1000;
    default:
        return 1017.63 / 1000;
    }
}

constexpr float getPhrSymbolDurationUs() {
    switch (DECAWAVE_TX_RATE_HZ) {
    case 110000:
        return 8205.13 / 1000;
    default:
        return 1025.64 / 1000;
    }
}

uint16_t InterlocTimeManager::getPreambleAirTimeUs() {
    float shrBitLength = PREAMBULE_SEQUENCE_LENGTH + START_FRAME_DELIMITER_LENGTH;
    float phrBitLength = PHY_HEADER_LENGTH;

    return (uint16_t)(shrBitLength * getShrSymbolDurationUs() +
                      phrBitLength * getPhrSymbolDurationUs());
}

uint16_t InterlocTimeManager::getReadWriteSPITimeUs(uint16_t bitLength) {
    return bitLength * 1000000 / SPI_SPEED_HZ;
}

uint16_t InterlocTimeManager::getAirTimeUs(uint16_t bitLength) {
    return bitLength * 1000000 / DECAWAVE_TX_RATE_HZ;
}

uint16_t InterlocTimeManager::computeAirTimeWithPreambleUs(uint16_t bitLength) {
    return getAirTimeUs(bitLength) + getPreambleAirTimeUs();
}
uint64_t InterlocTimeManager::getResponseTxTs_new(uint64_t startOfFrameTs) const {
    return (startOfFrameTs +
            UUS_TO_DWT_TIME *
                (m_pollToFirstResponseGuardUs +
                 (m_slotId - 1U) * (getReadWriteSPITimeUs(sizeof(UWBMessages::TWRResponse)) +
                                    RESPONSE_PROCESSING_GUARD + getPreambleAirTimeUs() +
                                    TIMEOUT_GUARD_US + RESPONSE_PROCESSING_GUARD_GUARD))) %
           UINT40_MAX;
}

void InterlocTimeManager::computeResponseRxTs_new(uint64_t startOfFrameTs) {
    for (uint8_t slotNb = 0; slotNb < m_numSlots; slotNb++) {
        m_responseRxTs_new[slotNb] =
            (startOfFrameTs +
             UUS_TO_DWT_TIME *
                 (m_pollToFirstResponseGuardUs +
                  (slotNb) * (getReadWriteSPITimeUs(sizeof(UWBMessages::TWRResponse)) +
                              RESPONSE_PROCESSING_GUARD + getPreambleAirTimeUs() +
                              TIMEOUT_GUARD_US + RESPONSE_PROCESSING_GUARD_GUARD) -
                  RX_BEFORE_TX_GUARD_US)) %
            UINT40_MAX;
    }
}

uint64_t InterlocTimeManager::getFinalTxTs_new(uint64_t startOfFrameTs) const {
    return (startOfFrameTs +
            UUS_TO_DWT_TIME *
                (m_pollToFirstResponseGuardUs +
                 (m_numSlots - 1U) * (getReadWriteSPITimeUs(sizeof(UWBMessages::TWRResponse)) +
                                      RESPONSE_PROCESSING_GUARD + getPreambleAirTimeUs() +
                                      TIMEOUT_GUARD_US + RESPONSE_PROCESSING_GUARD_GUARD) +
                 getTimeoutUs_new(m_responseAirTimeWithPreamble) + RESPONSE_PROCESSING_GUARD +
                 RESPONSE_TO_FINAL_GUARD)) %
           UINT40_MAX;

    //        (startOfFrameTs +
    //            UUS_TO_DWT_TIME *
    //                (m_pollToFirstResponseGuardUs +
    //                 m_numSlots * (getReadWriteSPITimeUs(sizeof(UWBMessages::TWRResponse)) +
    //                               RESPONSE_PROCESSING_GUARD + getPreambleAirTimeUs()))) %
    //           UINT40_MAX;
}

uint64_t InterlocTimeManager::getFinalRxTs_new(uint64_t startOfFrameTs) const {
    return (startOfFrameTs +
            UUS_TO_DWT_TIME *
                (m_pollToFirstResponseGuardUs +
                 (m_numSlots - 1U) * (getReadWriteSPITimeUs(sizeof(UWBMessages::TWRResponse)) +
                                      RESPONSE_PROCESSING_GUARD + getPreambleAirTimeUs() +
                                      TIMEOUT_GUARD_US + RESPONSE_PROCESSING_GUARD_GUARD) +
                 getTimeoutUs_new(m_responseAirTimeWithPreamble) + RESPONSE_PROCESSING_GUARD +
                 RESPONSE_TO_FINAL_GUARD - RX_BEFORE_TX_GUARD_US)) %
           UINT40_MAX;

    //    (startOfFrameTs +
    //     UUS_TO_DWT_TIME * (m_pollToFirstResponseGuardUs +
    //                        m_numSlots * (getReadWriteSPITimeUs(sizeof(UWBMessages::TWRResponse))
    //                        +
    //                                      RESPONSE_PROCESSING_GUARD + getPreambleAirTimeUs()) -
    //                        getPreambleAirTimeUs() - RX_BEFORE_TX_GUARD_US)) %
    //        UINT40_MAX;
}
uint16 InterlocTimeManager::getTimeoutUs_new(uint16_t msgAirTimeWithPreambleUs) {
    return msgAirTimeWithPreambleUs + TIMEOUT_GUARD_US;
}

uint64_t InterlocTimeManager::getSupposedNextFrameStart_new(uint64_t startOfFrameTs) const {
    return (startOfFrameTs + UUS_TO_DWT_TIME * getSuperFrameLengthUs_new()) % UINT40_MAX;
}

uint64_t InterlocTimeManager::getPollRxStartTs_new(uint64_t startOfFrameTs) const {
    return (getSupposedNextFrameStart_new(startOfFrameTs) -
            RX_BEFORE_TX_GUARD_US * UUS_TO_DWT_TIME) %
           UINT40_MAX;
}

uint64_t InterlocTimeManager::getPollTxStartTs_new(uint64_t startOfFrameTs) const {
    return (getSupposedNextFrameStart_new(startOfFrameTs)) % UINT40_MAX;
}

uint16_t InterlocTimeManager::getSyncTimeoutUs_new() const {
    uint32_t slotToSlotOffsetUs = getSuperFrameLengthUs_new();
    return slotToSlotOffsetUs + (m_bsp.generateRandomNumber() % 100) * 100;
}

uint16_t InterlocTimeManager::getSuperFrameLengthUs_new() const {
    return (getFinalRxTs_new(0) / UUS_TO_DWT_TIME + m_finalAirTimeWithPreamble +
            getReadWriteSPITimeUs(sizeof(UWBMessages::TWRFinal)) + FINAL_PROCESSING_GUARD);
}
