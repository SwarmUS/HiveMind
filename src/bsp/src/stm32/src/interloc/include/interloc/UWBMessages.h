#ifndef HIVE_MIND_UWBMESSAGES_H
#define HIVE_MIND_UWBMESSAGES_H

#include <cstdint>

#define UWB_BROADCAST_ADDRESS 0xFF

// TODO: Make configurable and merge with define in interloc lib
#define MAX_INTERLOC_SUBFRAMES 3

namespace UWBMessages {
    enum FrameType { BEACON = 0x0, DATA = 0x2, ACK = 0x3, MAC_COMMAND = 0x4 };
    enum AddressMode { SHORT_ADDRESS = 0x2 };
    enum FrameVersion { VERSION_0 = 0x0, VERSION_1 = 0x1 };
    enum FunctionCode : uint16_t {
        TWR_POLL = 0x61,
        TWR_RESPONSE = 0x50,
        TWR_FINAL = 0x69,
        ANGLE = 0x42
    };

    struct __attribute__((__packed__)) FrameControl {
        FrameType m_frameType : 3;
        uint8_t m_securityEnabled : 1;
        uint8_t m_framePending : 1;
        uint8_t m_ackRequest : 1;
        uint8_t m_panIdCompress : 1; // Always set to 1
        uint8_t m_reserved : 3;
        AddressMode m_destAddressMode : 2;
        uint8_t m_frameVersion : 2;
        AddressMode m_sourceAddressMode : 2;
    };

    struct __attribute__((__packed__)) MACHeader {
        FrameControl m_frameControl;
        uint8_t m_sequenceNumber;
        uint16_t m_destinationPanId;
        uint16_t m_destinationAddress;
        uint16_t m_sourceAddress;
    };

    struct __attribute__((__packed__)) DWFrame {
        MACHeader m_header;
        FunctionCode m_functionCode;
    };

    struct __attribute__((__packed__)) TWRPoll {
        DWFrame m_headerFrame;
        uint8_t m_superFrameInitiator;
        uint8_t m_currentFrameId;
    };

    struct __attribute__((__packed__)) TWRResponse {
        DWFrame m_headerFrame;
        uint8_t m_subFrameId;
    };

    struct __attribute__((__packed__)) TWRFinal {
        DWFrame m_headerFrame;
        uint64_t m_pollTxTs;
        uint64_t m_responseRxTs[MAX_INTERLOC_SUBFRAMES];
        uint64_t m_finalTxTs;
    };

    struct __attribute__((__packed__)) AngleMsg {
        DWFrame m_headerFrame;
        uint32_t m_messageId;
    };

} // namespace UWBMessages

#endif // HIVE_MIND_UWBMESSAGES_H
