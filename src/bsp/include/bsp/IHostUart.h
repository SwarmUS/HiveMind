#ifndef __IHOSTUART_H__
#define __IHOSTUART_H__

#include <cstdint>

#define HOST_UART_MAX_MESSAGE_LENGTH UINT16_MAX
#define HOST_UART_HEADER_LENGTH 6

class IHostUart {
  public:
    virtual ~IHostUart() = default;

    /**
     * @brief Sends a buffer to the Host via UART by prepending with the length and the CRC32
     * (non-blocking and thread-safe)
     * @param buffer Pointer to the data to send
     * @param length Number of bytes to send
     * @return True if transfer started. False otherwise.
     */
    virtual bool send(const uint8_t* buffer, uint16_t length) = 0;

    /**
     * @brief Receives up to an amount of data from the UART port
     * @param buffer Pointer to the buffer in which to put the received data
     * @param length Maximum length of data to receive
     * @return Number of bytes received or -1 in case of an error
     */
    virtual int32_t receive(uint8_t* buffer, uint16_t length) const = 0;

    /**
     * @brief Checks if driver is already busy transmitting data.
     * @return True if in use. False otherwise
     */
    virtual bool isBusy() const = 0;

    // TODO: Method to register a callback so we can be notified when a message is received.
    // (Depending on which notification scheme we choose to use)
};

#endif //__IHOSTUART_H__