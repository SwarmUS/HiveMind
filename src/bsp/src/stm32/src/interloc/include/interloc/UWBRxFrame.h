#ifndef __UWBRXFRAME_H__
#define __UWBRXFRAME_H__

#include "UWBMessages.h"
#include <array>

#define UWB_MAX_LENGTH 127
#define UWB_CRC_LENGTH 2
#define ACCUMULATOR_DATA_SIZE 5

enum class UWBRxStatus { ONGOING, FINISHED, TIMEOUT, ERROR };

struct UWBRxFrame {
    uint16_t m_length = 0;
    uint64_t m_rxTimestamp = 0;
    uint8_t m_sfdAngleRegister = 0;
    uint8_t m_firstPathAccumulator[ACCUMULATOR_DATA_SIZE]{};

    uint16_t m_stdNoise = 0;
    uint16_t m_fpAmpl1 = 0;
    uint16_t m_fpAmpl2 = 0;
    uint16_t m_fpAmpl3 = 0;
    uint16_t m_cirPwr = 0;
    uint16_t m_rxPreambleCount = 0;

    std::array<uint8_t, UWB_MAX_LENGTH - UWB_CRC_LENGTH> m_rxBuffer{};

    float getSFDAngle() const;
    float getAccumulatorAngle() const;
    float getLOSConfidence() const;
};

#endif //__UWBRXFRAME_H__
