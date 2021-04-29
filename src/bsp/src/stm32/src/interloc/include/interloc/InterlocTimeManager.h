#ifndef __INTERLOCTIMEMANAGER_H__
#define __INTERLOCTIMEMANAGER_H__

#include <bsp/IBSP.h>
#include <cstdint>

#define UINT40_MAX 0xFFFFFFFFFF

//#define RX_BEFORE_TX_GUARD_US 100U
//#define TIMEOUT_GUARD_US 1500U

#define POLL_TO_FIRST_RESPONSE_GUARD_US 900U
#define RESPONSE_TO_RESPONSE_GUARD_US 2000U
#define FINAL_TO_POLL_GUARD_US 2000U
#define RESPONSE_TO_FINAL_GUARD_US 900U

// TODO: Calculate these dynamically from the DW settings + number of agents
#define POLL_AIR_TIME_WITH_PREAMBLE_US 350U
#define RESPONSE_AIR_TIME_WITH_PREAMBLE_US 350U
#define FINAL_AIR_TIME_WITH_PREAMBLE_US 400U

// new guards
#define POLL_PROCESSING_GUARD 400U
#define RESPONSE_PROCESSING_GUARD 360U
#define RESPONSE_PROCESSING_GUARD_GUARD 50U
#define RESPONSE_TO_FINAL_GUARD 600U
#define FINAL_PROCESSING_GUARD 100U
#define RX_BEFORE_TX_GUARD_US 10U
#define TIMEOUT_GUARD_US 100U
#define DEAD_TIME 200U

class InterlocTimeManager {
  public:
    InterlocTimeManager(IBSP& bsp);

    void setNumSlots(uint16_t numSlots);
    void setSlodId(uint16_t slotId);

    uint64_t getPollTxTs(uint64_t lastSlotStartTs) const;
    static uint64_t getResponseTimeout();
    uint64_t getFinalTxTs(uint64_t pollTxTs) const;

    uint64_t getPollRxStartTs(uint64_t lastSlotStartTs) const;
    uint64_t getPollTimeout() const;
    uint64_t getResponseTxTs(uint64_t pollRxTs) const;
    uint64_t getFinalRxStartTs(uint64_t pollRxTs) const;
    static uint64_t getFinalTimeout();
    uint64_t getRespRxStartTime(uint64_t pollTxTs, uint8_t slotIdx);

    uint32_t getSyncTimeoutUs();

    static uint16_t getReadWriteSPITimeUs(uint16_t bitLength);
    static uint16_t getAirTimeUs(uint16_t bitLength);
    static uint16_t computeAirTimeWithPreambleUs(uint16_t bitLength);
    static uint16_t getPreambleAirTimeUs();
    void computeResponseRxTs_new(uint64_t startOfFrameTs);
    uint64_t getResponseTxTs_new(uint64_t startOfFrame) const;
    static uint16_t getTimeoutUs_new(uint16_t msgAirTimeWithPreambleUs);
    uint64_t getFinalRxTs_new(uint64_t startOfFrameTs) const;
    uint64_t getFinalTxTs_new(uint64_t startOfFrameTs) const;
    uint64_t getSupposedNextFrameStart_new(uint64_t startOfFrameTs) const;
    uint64_t getPollRxStartTs_new(uint64_t startOfFrameTs) const;
    uint64_t getPollTxStartTs_new(uint64_t startOfFrameTs) const;
    uint16_t getSyncTimeoutUs_new() const;
    uint16_t getSuperFrameLengthUs_new() const;

    // fixed length constants to be accessed by states
    uint16_t m_pollAirTimeWithPreamble;
    uint16_t m_responseAirTimeWithPreamble;
    uint16_t m_finalAirTimeWithPreamble;
    // variable length guards
    uint16_t m_pollToFirstResponseGuardUs;

    uint64_t m_respRxStartOffset[10];
    // addressable timestamps
    volatile uint64_t m_responseRxTs_new[10];

  private:
    IBSP& m_bsp;
    uint16_t m_numSlots;

    uint16_t m_slotId;
    // TODO set in function of m_numSlots;
    uint64_t m_slotToSlotOffsetDTU;
    uint64_t m_pollRxToResponseTxOffsetDTU;
    uint64_t m_pollTxToFinalTxOffsetDTU;

    uint64_t m_pollRxToFinalRxOffsetDTU;

    void updateTimings();
};

#endif //__INTERLOCTIMEMANAGER_H__
