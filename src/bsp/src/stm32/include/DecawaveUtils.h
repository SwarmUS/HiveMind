#ifndef __DECAWAVEUTILS_H__
#define __DECAWAVEUTILS_H__

#include "Decawave.h"

namespace DecawaveUtils {

    uint8_t getPreambleLength(UWBSpeed speed);
    uint8_t getPACSize(uint8_t preambleLength);
    uint8_t getPreambleCode(uint8_t channel);
    uint8_t getDWSpeed(UWBSpeed speed);
    uint16_t getSFDTimeout(uint8_t preambleLengthRegister,
                           uint8_t sfdLength,
                           uint8_t pacSizeRegister);

} // namespace DecawaveUtils

#endif //__DECAWAVEUTILS_H__
