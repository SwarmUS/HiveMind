#ifndef __IINTERLOCMESSAGEHANDLER_H__
#define __IINTERLOCMESSAGEHANDLER_H__

#include <bsp/InterlocUpdate.h>

class IInterlocMessageHandler {
  public:
    /**
     * @brief Processes the next message in the queue
     * @return True if successfully processed, false otherwise
     */
    virtual bool processMessage() = 0;

    /**
     * @brief Retrieves a flag to know if data dumps to the host should be done or not
     * @return True if enabled, false otherwise.
     */
    virtual bool getDumpEnabled() const = 0;

    /**
     * @brief Sends a history of interloc updates back to the host
     * @param updatesHistory Pointer to an array of interlocUpdates
     * @param updatesLength Number of items in the array
     * @return True if message was queued for send, false otherwise
     */
    virtual bool sendInterlocDump(InterlocUpdate* updatesHistory, uint8_t updatesLength) = 0;
};

#endif //__IINTERLOCMESSAGEHANDLER_H__
