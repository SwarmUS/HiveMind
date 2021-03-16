#ifndef __ISPIESP_H__
#define __ISPIESP_H__

#include "common/IProtobufStream.h"
#include <cstdint>

#define CRC32_SIZE sizeof(uint32_t)
#define ESP_SPI_MAX_MESSAGE_LENGTH (2048u - CRC32_SIZE)

class ISpiEsp : public IProtobufStream {
  public:
    virtual ~ISpiEsp() = default;

    /**
     * @brief Sends a buffer to the Esp via spi by appending with the length and the CRC32
     * (non-blocking and thread-safe)
     * @param buffer Pointer to the data to send
     * @param length Number of bytes to send
     * @return True if transfer started. False otherwise.
     */
    bool send(const uint8_t* buffer, uint16_t length) override = 0;

    /**
     * @brief Receives up to an amount of data from the SPI channel
     * @param buffer Pointer to the buffer in which to put the received data
     * @param length Maximum length of data to receive
     * @return True if success, false otherwise.
     */
    bool receive(uint8_t* buffer, uint16_t length) override = 0;

    /**
     * @brief Checks if driver is already busy transmitting data.
     * @return True if in use. False otherwise
     */
    virtual bool isBusy() const = 0;

    virtual bool isConnected() const = 0;
};

#endif // __ISPIESP_H__
