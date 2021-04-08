#ifndef HIVE_MIND_TWOWAYRANGING_H
#define HIVE_MIND_TWOWAYRANGING_H

#include "UWBMessages.h"
#include "UWBRxFrame.h"
#include <cstdint>

class TwoWayRanging {
  public:
    double calculateDistance() const;

    void deserializeFinal(UWBMessages::TWRFinal* finalMessage);
    void constructFinal(UWBMessages::TWRFinal* finalMessage, uint64_t finalTxTs);

    uint64_t m_pollTxTs;
    uint64_t m_pollRxTs;

    uint64_t m_responseTxTs;
    uint64_t m_responseRxTs[MAX_INTERLOC_SUBFRAMES];

    uint64_t m_finalTxTs;
    uint64_t m_finalRxTs;

    //    UWBRxFrame m_pollMessage;
    //    UWBMessages::TWRPoll m_pollMsg{};
    //    UWBMessages::TWRResponse m_responseMsg{};
    //    UWBMessages::TWRFinal m_finalMsg{};
};

#endif // HIVE_MIND_TWOWAYRANGING_H
