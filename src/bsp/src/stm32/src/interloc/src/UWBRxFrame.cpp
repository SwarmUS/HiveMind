#include "interloc/UWBRxFrame.h"
#include <cmath>

float UWBRxFrame::getSFDAngle() const {
    // RCPHASE register is only 7 bits, so mask off the highest bit
    return (((float)(m_sfdAngleRegister & 0x7F)) / 64.0F) * M_PI;
}

float UWBRxFrame::getAccumulatorAngle() const {
    // Accumulator data has a dummy byte at the first index
    int16_t accumulatorQ = *((int16_t*)(m_firstPathAccumulator + 1));
    int16_t accumulatorI = *((int16_t*)(m_firstPathAccumulator + 3));

    // TODO: Maybe change for LUT
    return atan2f((float)accumulatorI, (float(accumulatorQ)));
}

float UWBRxFrame::getLOSConfidence() const {
    // See DW1000 user manual p.45-46
    // https://www.decawave.com/sites/default/files/resources/dw1000_user_manual_2.11.pdf
    constexpr float A = 121.74; // We are using a 64Mhz PRF
    constexpr float Z = 131072; // 2^17

    float f1_2 = (float)m_fpAmpl1 * (float)m_fpAmpl1;
    float f2_2 = (float)m_fpAmpl2 * (float)m_fpAmpl2;
    float f3_2 = (float)m_fpAmpl3 * (float)m_fpAmpl3;
    float n_2 = (float)m_rxPreambleCount * (float)m_rxPreambleCount;

    volatile float firstPathPower = 10 * log10f((f1_2 + f2_2 + f3_2) / n_2) - A;
    volatile float receivePower = 10 * log10f(((float)m_cirPwr * Z) / n_2) - A;

    float powerDiff = receivePower - firstPathPower;

    if (powerDiff < 6) {
        return 1.0;
    }

    if (powerDiff > 10) {
        return 0.0;
    }

    // Linear interpolation between (6, 1.0) and (10, 0.0)
    volatile float x = -0.25F * powerDiff + 2.5F;
    return x;
}
