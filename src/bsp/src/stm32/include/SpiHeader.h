#ifndef SPI_HEADER_H
#define SPI_HEADER_H

#include <cstdint>

namespace EspHeader {

    struct EspSystemState {
        uint8_t connected : 1;
        uint8_t hasNeighborData : 1;
        uint8_t failedCrc : 1;
        uint8_t unused : 5;
    };

    struct StmSystemState {
        uint8_t hasHost : 1;
        uint8_t failedCrc : 1;
        uint8_t unused : 6;
    };

    union SystemState {
        EspSystemState espSystemState;
        StmSystemState stmSystemState;
        uint8_t rawValue;
    };

    struct __attribute__((__packed__)) Header {
        SystemState systemState;
        uint16_t txSizeBytes;
        uint16_t rxSizeBytes;
        uint16_t payloadSizeBytes;
        uint8_t crc8; // might be removed in the future
    };

    const uint8_t sizeBytes = sizeof(Header);

} // namespace EspHeader

#endif // SPI_HEADER_H
