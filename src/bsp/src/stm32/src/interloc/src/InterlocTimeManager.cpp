#include "interloc/InterlocTimeManager.h"
#include <interloc/Decawave.h>
#include <interloc/UWBMessages.h>

InterlocTimeManager::InterlocTimeManager(IBSP& bsp) :
    m_pollAirTimeWithPreambleUs(0),
    m_responseAirTimeWithPreambleUs(0),
    m_finalAirTimeWithPreambleUs(0),
    m_pollToFirstResponseGuardUs(0),
    m_bsp(bsp),
    m_numSlots(5),
    m_slotId(2)

{
    // keep these around for debug purposes.
    // could be deleted once integrated and tested with the HBx6
    /*
    m_pollAirTimeWithPreambleUs = computeAirTimeWithPreambleUs(sizeof(UWBMessages::TWRPoll) << 3);
    m_responseAirTimeWithPreambleUs =
        computeAirTimeWithPreambleUs(sizeof(UWBMessages::TWRResponse) << 3);
    m_finalAirTimeWithPreambleUs = computeAirTimeWithPreambleUs(sizeof(UWBMessages::TWRFinal) << 3);
    m_pollToFirstResponseGuardUs = getReadWriteSPITimeUs(sizeof(UWBMessages::TWRPoll)) +
                                   POLL_PROCESSING_GUARD + getPreambleAirTimeUs();

    uint64_t startOfFrameTs = 0; // Absolute reference in all timings

    computeResponseRxTs(startOfFrameTs);

    volatile uint64_t responseTxTs[5];
    for (unsigned int i = 1; i < 6; i++) {
        m_slotId = i;
        responseTxTs[i - 1] = getResponseTxTs(startOfFrameTs);
    }
    m_slotId = 2;
    volatile uint64_t finalRxTs = getFinalRxTs(startOfFrameTs);
    volatile uint64_t finalTxTs = getFinalTxTs(startOfFrameTs);
    volatile uint16_t pollTO = getTimeoutUs(m_pollAirTimeWithPreambleUs);
    volatile uint16_t responseTO = getTimeoutUs(m_responseAirTimeWithPreambleUs);
    volatile uint16_t finalTO = getTimeoutUs(m_finalAirTimeWithPreambleUs);
    volatile uint64_t newPollTs = getPollRxStartTs(startOfFrameTs);

    responseTxTs[0]++;
    finalRxTs++;
    finalTxTs++;
    m_responseRxTs[9] = 0;
    pollTO++;
    responseTO++;
    finalTO++;
    newPollTs++;
*/
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
    // compute constant values air time with their preamble
    m_pollAirTimeWithPreambleUs = computeAirTimeWithPreambleUs(sizeof(UWBMessages::TWRPoll) << 3);
    m_responseAirTimeWithPreambleUs =
        computeAirTimeWithPreambleUs(sizeof(UWBMessages::TWRResponse) << 3);
    m_finalAirTimeWithPreambleUs = computeAirTimeWithPreambleUs(sizeof(UWBMessages::TWRFinal) << 3);
    m_pollToFirstResponseGuardUs = getReadWriteSPITimeUs(sizeof(UWBMessages::TWRPoll)) +
                                   POLL_PROCESSING_GUARD + getPreambleAirTimeUs();
}

float getShrSymbolDurationUs() {
    // see table 13 in datasheet
    // see table 61 in user manual
    switch (DecawaveUtils::getPreambleCode(UWBChannel::DW_CHANNEL)) {
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
        return 993.59 / 1000;
    default:
        return 1017.63 / 1000;
    }
}

constexpr float getPhrSymbolDurationUs() {
    // see table 13 in datasheet
    switch (UWBSpeed::DW_SPEED) {
    case UWBSpeed::SPEED_110K:
        return 8205.13 / 1000;
    default:
        return 1025.64 / 1000;
    }
}
uint8_t getSFDLength() {
    // see user manual p.217
    switch (UWBSpeed::DW_SPEED) {
    case UWBSpeed::SPEED_110K:
        return 64;
    default:
        return 8;
    }
}

uint16_t InterlocTimeManager::getPreambleAirTimeUs() {
    float shrBitLength = DecawaveUtils::getPreambleLength(UWBSpeed::DW_SPEED) + getSFDLength();
    float phrBitLength = PHY_HEADER_LENGTH;

    return (uint16_t)(shrBitLength * getShrSymbolDurationUs() +
                      phrBitLength * getPhrSymbolDurationUs());
}

uint16_t InterlocTimeManager::getReadWriteSPITimeUs(uint16_t bitLength) {
    return bitLength * 1000000 / SPI_SPEED_HZ;
}

uint16_t InterlocTimeManager::getAirTimeUs(uint16_t bitLength) {
    switch (UWBSpeed::DW_SPEED) {
    case UWBSpeed::SPEED_6M8:
        return bitLength * 1000000 / 6.8e6;
    case UWBSpeed::SPEED_850K:
        return bitLength * 1000000 / 850e3;
    default:
        return bitLength * 1000000 / 110e3;
    }
}

uint16_t InterlocTimeManager::computeAirTimeWithPreambleUs(uint16_t bitLength) {
    return getAirTimeUs(bitLength) + getPreambleAirTimeUs();
}
uint64_t InterlocTimeManager::getResponseTxTs(uint64_t startOfFrame) const {
    return (startOfFrame +
            UUS_TO_DWT_TIME *
                (getPreambleAirTimeUs() + m_pollToFirstResponseGuardUs +
                 (m_slotId - 1U) * (getReadWriteSPITimeUs(sizeof(UWBMessages::TWRResponse)) +
                                    RESPONSE_PROCESSING_GUARD + getPreambleAirTimeUs() +
                                    TIMEOUT_GUARD_US + RESPONSE_PROCESSING_GUARD_GUARD))) %
           UINT40_MAX;
}

void InterlocTimeManager::computeResponseRxTs(uint64_t startOfFrameTs) {
    for (uint8_t slotNb = 0; slotNb < m_numSlots; slotNb++) {
        m_responseRxTs[slotNb] =
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

uint64_t InterlocTimeManager::getFinalTxTs(uint64_t startOfFrameTs) const {
    return (startOfFrameTs +
            UUS_TO_DWT_TIME *
                (getPreambleAirTimeUs() + m_pollToFirstResponseGuardUs +
                 (m_numSlots - 1U) * (getReadWriteSPITimeUs(sizeof(UWBMessages::TWRResponse)) +
                                      RESPONSE_PROCESSING_GUARD + getPreambleAirTimeUs() +
                                      TIMEOUT_GUARD_US + RESPONSE_PROCESSING_GUARD_GUARD) +
                 getTimeoutUs(m_responseAirTimeWithPreambleUs) + RESPONSE_PROCESSING_GUARD +
                 RESPONSE_TO_FINAL_GUARD)) %
           UINT40_MAX;
}

uint64_t InterlocTimeManager::getFinalRxTs(uint64_t startOfFrameTs) const {
    return (startOfFrameTs +
            UUS_TO_DWT_TIME *
                (m_pollToFirstResponseGuardUs +
                 (m_numSlots - 1U) * (getReadWriteSPITimeUs(sizeof(UWBMessages::TWRResponse)) +
                                      RESPONSE_PROCESSING_GUARD + getPreambleAirTimeUs() +
                                      TIMEOUT_GUARD_US + RESPONSE_PROCESSING_GUARD_GUARD) +
                 getTimeoutUs(m_responseAirTimeWithPreambleUs) + RESPONSE_PROCESSING_GUARD +
                 RESPONSE_TO_FINAL_GUARD - RX_BEFORE_TX_GUARD_US)) %
           UINT40_MAX;
}
uint16 InterlocTimeManager::getTimeoutUs(uint16_t msgAirTimeWithPreambleUs) {
    return msgAirTimeWithPreambleUs + TIMEOUT_GUARD_US;
}

uint64_t InterlocTimeManager::getSupposedNextFrameStart(uint64_t startOfFrameTs) const {
    return (startOfFrameTs + UUS_TO_DWT_TIME * getSuperFrameLengthUs()) % UINT40_MAX;
}

uint64_t InterlocTimeManager::getPollRxStartTs(uint64_t startOfFrameTs) const {
    return (getSupposedNextFrameStart(startOfFrameTs) - RX_BEFORE_TX_GUARD_US * UUS_TO_DWT_TIME) %
           UINT40_MAX;
}

uint64_t InterlocTimeManager::getPollTxStartTs(uint64_t startOfFrameTs) const {
    return (UUS_TO_DWT_TIME * getPreambleAirTimeUs() + getSupposedNextFrameStart(startOfFrameTs)) %
           UINT40_MAX;
}

uint16_t InterlocTimeManager::getSyncTimeoutUs() const {
    uint32_t slotToSlotOffsetUs = getSuperFrameLengthUs();
    return slotToSlotOffsetUs + (m_bsp.generateRandomNumber() % 100) * 100;
}

uint16_t InterlocTimeManager::getSuperFrameLengthUs() const {
    return (getFinalRxTs(0) / UUS_TO_DWT_TIME + m_finalAirTimeWithPreambleUs +
            getReadWriteSPITimeUs(sizeof(UWBMessages::TWRFinal)) + FINAL_PROCESSING_GUARD);
}
