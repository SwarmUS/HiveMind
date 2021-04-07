#ifndef HIVE_MIND_TWOWAYRANGING_H
#define HIVE_MIND_TWOWAYRANGING_H

#include "UWBMessages.h"
#include <cstdint>

class TwoWayRanging {
    double calculateDistance() const;

    void deserializeFinal(UWBMessages::TWRFinal* finalMessage);
    void constructFinal(UWBMessages::TWRFinal* finalMessage, uint64_t finalTxTs);

  private:
    uint64_t m_pollTxTs;

    uint64_t m_pollRxTs;

    uint64_t m_responseTxTs;
    uint64_t m_responseRxTs;

    uint64_t m_finalTxTs;
    uint64_t m_finalRxTs;
};

#endif // HIVE_MIND_TWOWAYRANGING_H
