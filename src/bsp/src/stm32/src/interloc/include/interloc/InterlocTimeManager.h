#ifndef __INTERLOCTIMEMANAGER_H__
#define __INTERLOCTIMEMANAGER_H__

#include <cstdint>

#define NUM_SLOTS 3
#define MY_SLOT 2

#define RX_BEFORE_TX_GUARD_US 50
#define TIMEOUT_GUARD_US 50

#define POLL_TO_FIRST_RESPONSE_GUARD_US 100
#define RESPONSE_RX_TO_RESPONSE_TX_GUARD_US 100
#define RESPONSE_RX_TO_FINAL_TX_GUARD_US 100
#define FINAL_TO_POLL_GUARD_US 100

// TODO: Calculate these dynamically from the DW settings + number of agents
#define RESPONSE_AIR_TIME_US 106
#define POLL_AIR_TIME_US 106
#define FINAL_AIR_TIME_US 176

#define RESPONSE_TO_RESPONSE_DELAY RESPONSE_RX_TO_RESPONSE_TX_GUARD_US + RESPONSE_AIR_TIME_US

class InterlocTimeManager {
  public:
    void updateTimings();

    uint64_t getPollTxTs(uint64_t lastSlotStartTs);
    static uint64_t getResponseTimeout();
    uint64_t getFinalTxTs(uint64_t pollTxTs) const;

    uint64_t getPollRxStartTs(uint64_t lastSlotStartTs);
    static uint64_t getPollTimeout();
    uint64_t getResponseTxTs(uint64_t pollRxTs) const;
    uint64_t getFinalRxStartTs(uint64_t pollRxTs) const;
    static uint64_t getFinalTimeout();

  private:
    uint64_t m_slotToSlotOffsetDTU;
    uint64_t m_pollRxToResponseTxOffsetDTU;
    uint64_t m_pollTxToFinalTxOffsetDTU;
    uint64_t m_pollRxToFinalRxOffsetDTU;
};

#endif //__INTERLOCTIMEMANAGER_H__
