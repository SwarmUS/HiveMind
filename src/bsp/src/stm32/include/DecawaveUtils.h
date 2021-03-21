#ifndef __DECAWAVEUTILS_H__
#define __DECAWAVEUTILS_H__

#include "Decawave.h"

namespace DecawaveUtils {

    /**
     * @brief Returns the DW1000 macro for preamble length given a speed
     * @param speed Data rate
     * @return The preamble length macro
     */
    uint8_t getPreambleLength(UWBSpeed speed);

    /**
     * @brief Returns the DW1000 macro for PAC size given a preamble length
     * @param preambleLength The preamble length macro
     * @return The PAC size macro
     */
    uint8_t getPACSize(uint8_t preambleLength);

    /**
     * @brief Returns the preamble code for a given channel
     * @param channel UWB channel
     * @return Preamble code index
     */
    uint8_t getPreambleCode(UWBChannel channel);

    /**
     * @brief Calculates a SFD timeout based on other configurations
     * @param preambleLengthRegister Macro value for preamble length
     * @param sfdLength SFD length (16 or 64)
     * @param pacSizeRegister Macro value for PAC size
     * @return The SFD timeout
     */
    uint16_t getSFDTimeout(uint8_t preambleLengthRegister,
                           uint8_t sfdLength,
                           uint8_t pacSizeRegister);

} // namespace DecawaveUtils

#endif //__DECAWAVEUTILS_H__
