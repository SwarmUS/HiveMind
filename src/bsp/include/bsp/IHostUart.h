#ifndef __IHOSTUART_H__
#define __IHOSTUART_H__

#include <common/IProtobufStream.h>
#include <cstdint>

#define HOST_UART_MAX_MESSAGE_LENGTH 1024
#define HOST_UART_STREAM_SIZE 4096
#define HOST_UART_HEADER_LENGTH 6

class IHostUart : public IProtobufStream {
  public:
    ~IHostUart() override = default;

    /**
     * @brief Sends a buffer to the Host via UART by prepending with the length and the CRC32
     * (non-blocking and thread-safe)
     * @param buffer Pointer to the data to send
     * @param length Number of bytes to send
     * @return True if transfer started. False otherwise.
     */
    bool send(const uint8_t* buffer, uint16_t length) override = 0;

    /**
     * @brief Receives up to an amount of data from the UART port
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
};

#endif //__IHOSTUART_H__
