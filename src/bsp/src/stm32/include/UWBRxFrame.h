#ifndef __UWBRXFRAME_H__
#define __UWBRXFRAME_H__

#include "UWBMessages.h"
#include <array>

#define UWB_MAX_LENGTH 127
#define UWB_CRC_LENGTH 2

enum class UWBRxStatus { ONGOING, FINISHED, TIMEOUT, ERROR };

struct UWBRxFrame {
    uint16_t m_length = 0;
    uint64_t m_rxTimestamp = 0;
    UWBRxStatus m_status = UWBRxStatus::ONGOING;
    uint32_t m_statusReg = 0;
    uint8_t m_sfdAngleRegister = 0;
    uint8_t m_firstPathAccumulator[5]{};

    std::array<uint8_t, UWB_MAX_LENGTH - UWB_CRC_LENGTH> m_rxBuffer{};
    UWBMessages::DWFrame* m_frame = reinterpret_cast<UWBMessages::DWFrame*>(m_rxBuffer.data());
    uint8_t* m_payload = m_rxBuffer.data() + sizeof(UWBMessages::DWFrame);

    float getSFDAngle() const;
    float getAccumulatorAngle();
};

#endif //__UWBRXFRAME_H__
