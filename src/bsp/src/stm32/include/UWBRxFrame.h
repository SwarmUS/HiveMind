#ifndef __UWBRXFRAME_H__
#define __UWBRXFRAME_H__

#include <array>

#define UWB_MAX_LENGTH 125 // 127 - 2 CRC bytes

enum class UWBRxStatus { ONGOING, FINISHED, TIMEOUT, ERROR };

struct UWBRxFrame {
    uint16_t m_length;
    uint64_t m_rxTimestamp;
    UWBRxStatus m_status;
    uint32_t m_statusReg;

    std::array<uint8_t, UWB_MAX_LENGTH> m_rxBuffer;
};

#endif //__UWBRXFRAME_H__
