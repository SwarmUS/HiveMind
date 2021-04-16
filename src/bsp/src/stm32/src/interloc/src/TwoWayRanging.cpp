#include "interloc/TwoWayRanging.h"
#include <cstring>
#include <deca_device_api.h>
#include <interloc/Decawave.h>

#define MAX_POSSIBLE_DISTANCE 500

void TwoWayRanging::deserializeFinal(UWBMessages::TWRFinal* finalMessage) {
    // Need to do memcpys instead of assignments as the values aren't 64bit aligned
    memcpy(&m_pollTxTs, &(finalMessage->m_pollTxTs), sizeof(m_pollTxTs));
    for (unsigned int i = 0; i < MAX_INTERLOC_SUBFRAMES; i++) {
        memcpy(&m_responseRxTs[i], &(finalMessage->m_responseRxTs[i]), sizeof(m_responseRxTs));
    }
    memcpy(&m_finalTxTs, &(finalMessage->m_finalTxTs), sizeof(m_finalTxTs));
}

void TwoWayRanging::constructFinal(UWBMessages::TWRFinal* finalMessage, uint64_t finalTxTs) {
    m_finalTxTs = finalTxTs;

    DecawaveUtils::tsToBytes((uint8_t*)(&finalMessage->m_pollTxTs), m_pollTxTs);
    for (unsigned int i = 0; i < MAX_INTERLOC_SUBFRAMES; i++) {
        DecawaveUtils::tsToBytes((uint8_t*)(&finalMessage->m_responseRxTs[i]), m_responseRxTs[i]);
    }
    DecawaveUtils::tsToBytes((uint8_t*)(&finalMessage->m_finalTxTs), m_finalTxTs);
}

std::optional<double> TwoWayRanging::calculateDistance(uint16_t slotId) const {
    uint64_t tRound1 = m_responseRxTs[slotId - 1] - m_pollTxTs;
    uint32_t tRound2 = (uint32_t)m_finalRxTs - (uint32_t)m_responseTxTs;
    uint32_t tReply1 = (uint32_t)m_responseTxTs - (uint32_t)m_pollRxTs;
    uint64_t tReply2 = m_finalTxTs - m_responseRxTs[slotId - 1];

    uint64_t tofDtu =
        (tRound1 * tRound2 - tReply1 * tReply2) / (tRound1 + tRound2 + tReply1 + tReply2);

    double tof = tofDtu * DWT_TIME_UNITS;
    double distance = tof * SPEED_OF_LIGHT;

    if (distance < MAX_POSSIBLE_DISTANCE) {
        return distance;
    }

    return {};
}
