#include "interloc/TwoWayRanging.h"
#include <deca_device_api.h>
#include <interloc/Decawave.h>

void TwoWayRanging::deserializeFinal(UWBMessages::TWRFinal* finalMessage) {
    m_pollTxTs = finalMessage->m_pollTxTs;
    m_responseRxTs = finalMessage->m_responseRxTs;
    m_finalTxTs = finalMessage->m_finalTxTs;
}

void TwoWayRanging::constructFinal(UWBMessages::TWRFinal* finalMessage, uint64_t finalTxTs) {
    m_finalTxTs = finalTxTs;

    DecawaveUtils::tsToBytes((uint8_t*)(&finalMessage->m_pollTxTs), m_pollTxTs);
    DecawaveUtils::tsToBytes((uint8_t*)(&finalMessage->m_responseRxTs), m_responseRxTs);
    DecawaveUtils::tsToBytes((uint8_t*)(&finalMessage->m_finalTxTs), m_finalTxTs);
}

double TwoWayRanging::calculateDistance() const {
    uint64_t tRound1 = m_responseRxTs - m_pollTxTs;
    uint32_t tRound2 = m_finalRxTs - (uint32_t)m_responseTxTs;
    uint64_t tReply1 = m_finalTxTs - m_responseRxTs;
    uint32_t tReply2 = (uint32_t)m_responseTxTs - (uint32_t)m_pollRxTs;

    uint64_t tofDtu =
        (tRound1 * tRound2 - tReply1 * tReply2) / (tRound1 + tRound2 + tReply1 + tReply2);

    double tof = tofDtu * DWT_TIME_UNITS;
    return tof * SPEED_OF_LIGHT;
}
