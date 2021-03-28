#include "UWBRxFrame.h"
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