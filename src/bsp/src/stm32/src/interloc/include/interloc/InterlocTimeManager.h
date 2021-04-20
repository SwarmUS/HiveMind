#ifndef __INTERLOCTIMEMANAGER_H__
#define __INTERLOCTIMEMANAGER_H__

#include <bsp/IBSP.h>
#include <cstdint>

#define UINT40_MAX 0xFFFFFFFFFF

#define RX_BEFORE_TX_GUARD_US 100U
#define TIMEOUT_GUARD_US 1500U

#define POLL_TO_FIRST_RESPONSE_GUARD_US 900U
#define RESPONSE_TO_RESPONSE_GUARD_US 2000U
#define FINAL_TO_POLL_GUARD_US 2000U
#define RESPONSE_TO_FINAL_GUARD_US 900U

// TODO: Calculate these dynamically from the DW settings + number of agents
#define POLL_AIR_TIME_WITH_PREAMBLE_US 350U
#define RESPONSE_AIR_TIME_WITH_PREAMBLE_US 350U
#define FINAL_AIR_TIME_WITH_PREAMBLE_US 400U

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
    uint64_t getResponseTxTs(uint64_t slotIdx) const;
    uint64_t getFinalRxStartTs(uint64_t pollRxTs) const;
    static uint64_t getFinalTimeout();
    uint64_t getRespRxStartTime(uint64_t pollTxTs, uint8_t slotIdx);

    uint32_t getSyncTimeoutUs();

  private:
    IBSP& m_bsp;

    uint16_t m_numSlots;
    uint16_t m_slotId;

    // TODO set in function of m_numSlots;
    uint64_t m_RespRxStartOffset[10];
    uint64_t m_slotToSlotOffsetDTU;
    uint64_t m_pollRxToResponseTxOffsetDTU;
    uint64_t m_pollTxToFinalTxOffsetDTU;
    uint64_t m_pollRxToFinalRxOffsetDTU;

    void updateTimings();
};

#endif //__INTERLOCTIMEMANAGER_H__
