#include "interloc/TwoWayRanging.h"
#include <cstring>
#include <deca_device_api.h>
#include <interloc/Decawave.h>

#define MAX_POSSIBLE_DISTANCE 500

void TwoWayRanging::deserializeFinal(UWBMessages::TWRFinal* finalMessage) {
    memcpy(&m_pollTxTs, &(finalMessage->m_pollTxTs), sizeof(m_pollTxTs));
    memcpy(&m_responseRxTs, &(finalMessage->m_responseRxTs), sizeof(m_responseRxTs));
    memcpy(&m_finalTxTs, &(finalMessage->m_finalTxTs), sizeof(m_finalTxTs));
    //    m_pollTxTs = ((UWBMessages::TWRFinal*)finalMessage)->m_pollTxTs;
    //    m_responseRxTs = ((UWBMessages::TWRFinal*)finalMessage)->m_responseRxTs;
    //    m_finalTxTs = ((UWBMessages::TWRFinal*)finalMessage)->m_finalTxTs;
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
    uint32_t tReply1 = (uint32_t)m_responseTxTs - (uint32_t)m_pollRxTs;
    uint64_t tReply2 = m_finalTxTs - m_responseRxTs;

    uint64_t tofDtu =
        (tRound1 * tRound2 - tReply1 * tReply2) / (tRound1 + tRound2 + tReply1 + tReply2);

    double tof = tofDtu * DWT_TIME_UNITS;
    tof *= SPEED_OF_LIGHT;

    return tof < MAX_POSSIBLE_DISTANCE ? tof : 0;
}
