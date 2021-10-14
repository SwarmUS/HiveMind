#ifndef __IINTERLOCMESSAGEHANDLER_H__
#define __IINTERLOCMESSAGEHANDLER_H__

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
};

#endif //__IINTERLOCMESSAGEHANDLER_H__
