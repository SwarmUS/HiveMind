#ifndef __INTERLOCTIMEMANAGER_H__
#define __INTERLOCTIMEMANAGER_H__

#include <bsp/IBSP.h>
#include <cstdint>

#define UINT40_MAX 0xFFFFFFFFFF

#define POLL_PROCESSING_GUARD 400U
#define RESPONSE_PROCESSING_GUARD 360U
#define RESPONSE_PROCESSING_GUARD_GUARD 50U
#define RESPONSE_TO_FINAL_GUARD 600U
#define FINAL_PROCESSING_GUARD 700U
#define RX_BEFORE_TX_GUARD_US 10U
#define TIMEOUT_GUARD_US 100U
#define DEAD_TIME 200U

class InterlocTimeManager {
  public:
    InterlocTimeManager(IBSP& bsp);

    void setNumSlots(uint16_t numSlots);
    void setSlodId(uint16_t slotId);

    static uint16_t getReadWriteSPITimeUs(uint16_t bitLength);
    static uint16_t getAirTimeUs(uint16_t bitLength);
    static uint16_t computeAirTimeWithPreambleUs(uint16_t bitLength);
    static uint16_t getPreambleAirTimeUs();
    void computeResponseRxTs(uint64_t startOfFrameTs);
    uint64_t getResponseTxTs(uint64_t startOfFrame) const;
    static uint16_t getTimeoutUs(uint16_t msgAirTimeWithPreambleUs);
    uint64_t getFinalRxTs(uint64_t startOfFrameTs) const;
    uint64_t getFinalTxTs(uint64_t startOfFrameTs) const;
    uint64_t getSupposedNextFrameStart(uint64_t startOfFrameTs) const;
    uint64_t getPollRxStartTs(uint64_t startOfFrameTs) const;
    uint64_t getPollTxStartTs(uint64_t startOfFrameTs) const;
    uint16_t getSyncTimeoutUs() const;
    uint16_t getSuperFrameLengthUs() const;

    // fixed length constants to be accessed by states
    uint16_t m_pollAirTimeWithPreambleUs;
    uint16_t m_responseAirTimeWithPreambleUs;
    uint16_t m_finalAirTimeWithPreambleUs;
    uint16_t m_pollToFirstResponseGuardUs;

    // addressable timestamps
    volatile uint64_t m_responseRxTs[10];

  private:
    IBSP& m_bsp;

    uint16_t m_numSlots; // MAX_INTERLOC_SUBFRAMES
    uint16_t m_slotId;

    void updateTimings();
};

#endif //__INTERLOCTIMEMANAGER_H__
