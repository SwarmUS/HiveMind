#ifndef HIVE_MIND_IUSB_H
#define HIVE_MIND_IUSB_H

#include "ICommInterface.h"

class IUSB : public ICommInterface {
  public:
    virtual ~IUSB() = default;
    /**
     * @brief Sends a buffer via USB
     * (non-blocking and thread-safe)
     * @param buffer Pointer to the data to send
     * @param length Number of bytes to send
     * @return True if transfer Completed. False otherwise. */
    virtual bool send(const uint8_t* buffer, uint16_t length) = 0;

    /**
     * @brief Receives up to an amount of data from the USB port
     * @param buffer Pointer to the buffer in which to put the received data
     * @param length Maximum length of data to receive
     * @return True if success, false otherwise. */
    virtual bool receive(uint8_t* buffer, uint16_t length) = 0;

    /**
     * @brief Tells if client is connected to the port
     * @return True if connected, else otherwise */
    virtual bool isConnected() const = 0;
};
#endif // HIVE_MIND_IUSB_H
