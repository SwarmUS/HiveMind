#include "DecawaveUtils.h"
#include "deca_device_api.h"

uint8_t DecawaveUtils::getPreambleLength(UWBSpeed speed) {
    // Use biggest preamble length for given speed as higher length means higher accuracy
    // DW1000 user manual section 9.3 p.210

    switch (speed) {
    case UWBSpeed::SPEED_6M8:
        return DWT_PLEN_256;

    case UWBSpeed::SPEED_850K:
        return DWT_PLEN_1024;

    case UWBSpeed::SPEED_110K:
        return DWT_PLEN_1024;
    }

    return 0;
}

uint8_t DecawaveUtils::getPACSize(uint8_t preambleLength) {
    // DW1000 user manual section 4.1.1 p.32

    switch (preambleLength) {
    case DWT_PLEN_64:
    case DWT_PLEN_128:
        return DWT_PAC8;

    case DWT_PLEN_256:
    case DWT_PLEN_512:
        return DWT_PAC16;

    case DWT_PLEN_1024:
        return DWT_PAC32;

    default:
        return DWT_PAC64;
    }
}

uint8_t DecawaveUtils::getPreambleCode(uint8_t channel) {
    // Choose lowest code for a given channel
    // DW user manual section 10.5 p.218
    switch (channel) {
    case 4:
    case 7:
        return 17;

    default:
        return 9;
    }
}

uint8_t DecawaveUtils::getDWSpeed(UWBSpeed speed) {
    switch (speed) {
    case UWBSpeed::SPEED_110K:
        return DWT_BR_110K;

    case UWBSpeed::SPEED_850K:
        return DWT_BR_850K;

    case UWBSpeed::SPEED_6M8:
        return DWT_BR_6M8;
    }

    return 0;
}

uint16_t DecawaveUtils::getSFDTimeout(uint8_t preambleLengthRegister,
                                      uint8_t sfdLength,
                                      uint8_t pacSizeRegister) {
    // SFDTimeout = preamble length + 1 + SFDLength - PAC size
    uint16_t preambleLength = 0;
    uint8_t pacSize = 0;

    switch (preambleLengthRegister) {
    case DWT_PLEN_256:
        preambleLength = 256;
        break;

    case DWT_PLEN_1024:
        preambleLength = 1024;
        break;

    case DWT_PLEN_4096:
        preambleLength = 4096;
        break;
    }

    switch (pacSizeRegister) {
    case DWT_PAC8:
        pacSize = 8;
        break;

    case DWT_PAC16:
        pacSize = 16;
        break;

    case DWT_PAC32:
        pacSize = 32;
        break;

    case DWT_PAC64:
        pacSize = 64;
        break;
    }

    return preambleLength + 1 + sfdLength - pacSize;
}