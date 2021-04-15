#ifndef __INTERLOCTIMEMANAGER_H__
#define __INTERLOCTIMEMANAGER_H__

#include <bsp/IBSP.h>
#include <cstdint>

#define RX_BEFORE_TX_GUARD_US 50
#define TIMEOUT_GUARD_US 50

#define POLL_TO_FIRST_RESPONSE_GUARD_US 100
#define RESPONSE_RX_TO_RESPONSE_TX_GUARD_US 370
#define FINAL_TO_POLL_GUARD_US 100

// TODO: Calculate these dynamically from the DW settings + number of agents
#define RESPONSE_AIR_TIME_WITH_PREAMBLE_US 170
#define POLL_AIR_TIME_WITH_PREAMBLE_US 170
#define FINAL_AIR_TIME_WITH_PREAMBLE_US 240

class InterlocTimeManager {
  public:
    InterlocTimeManager(IBSP& bsp);

    void setNumSlots(uint16_t numSlots);
    void setSlodId(uint16_t slotId);

    uint64_t getPollTxTs(uint64_t lastSlotStartTs) const;
    static uint64_t getResponseTimeout();
    uint64_t getFinalTxTs(uint64_t pollTxTs) const;

    uint64_t getPollRxStartTs(uint64_t lastSlotStartTs) const;
    static uint64_t getPollTimeout();
    uint64_t getResponseTxTs(uint64_t pollRxTs) const;
    uint64_t getFinalRxStartTs(uint64_t pollRxTs) const;
    static uint64_t getFinalTimeout();

    uint32_t getSyncTimeoutUs();

  private:
    IBSP& m_bsp;

    uint16_t m_numSlots;
    uint16_t m_slotId;

    uint64_t m_slotToSlotOffsetDTU;
    uint64_t m_pollRxToResponseTxOffsetDTU;
    uint64_t m_pollTxToFinalTxOffsetDTU;
    uint64_t m_pollRxToFinalRxOffsetDTU;

    void updateTimings();
};

#endif //__INTERLOCTIMEMANAGER_H__
