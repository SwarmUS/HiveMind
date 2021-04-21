#ifndef __DECAWAVEUTILS_H__
#define __DECAWAVEUTILS_H__

#include "UWBRxFrame.h"
#include <cstdint>
#include <deca_device_api.h>

enum class UWBSpeed { SPEED_110K = DWT_BR_110K, SPEED_850K = DWT_BR_850K, SPEED_6M8 = DWT_BR_6M8 };
enum class UWBChannel {
    CHANNEL_1 = 1,
    CHANNEL_2 = 2,
    CHANNEL_3 = 3,
    CHANNEL_4 = 4,
    CHANNEL_5 = 5,
    CHANNEL_7 = 7
};

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

    /**
     * @brief Converts a timestamp to it's equivalent in bytes
     * @param tsField Destination of the converted timestamp
     * @param ts Timestamp to convert
     */
    void tsToBytes(uint8_t* tsField, uint64_t ts);

    /**
     * @brief Checks if the frame is a TWRResonse
     * @param rxFrame Received frame to check
     * @return True if the rxFrame is a TWRResonse
     */
    bool isFrameResponse(const UWBRxFrame& rxFrame);

    /**
     * @brief Checks if the frame is a TWRFinal
     * @param rxFrame Received frame to check
     * @return True if the rxFrame is a TWRFinal
     */
    bool isFrameFinal(const UWBRxFrame& rxFrame);

    /**
     * @brief Checks if the frame is a TWRPoll
     * @param rxFrame Received frame to check
     * @return True if the rxFrame is a TWRPoll
     */
    bool isFramePoll(const UWBRxFrame& rxFrame);

} // namespace DecawaveUtils

#endif //__DECAWAVEUTILS_H__
