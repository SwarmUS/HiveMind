#ifndef SPI_HEADER_H
#define SPI_HEADER_H

#include <cstdint>
namespace EspSpi {

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

    struct HeaderStruct {
        SystemState systemState;
        uint8_t txSizeWord;
        uint8_t rxSizeWord;
        uint8_t crc8; // might be removed in the future
    };

    const uint8_t headerSize = sizeof(HeaderStruct);

    union Header {
        HeaderStruct headerStruct;
        uint8_t bytes[headerSize];
    };
} // namespace EspSpi

#endif // SPI_HEADER_H
