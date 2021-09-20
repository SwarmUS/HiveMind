#ifndef __IBITTYBUZZMESSAGEHANDLER_H_
#define __IBITTYBUZZMESSAGEHANDLER_H_

#include <cstdint>

/**
 *@brief Handles messages destined to the bbvm*/
class IBittyBuzzMessageHandler {
  public:
    virtual ~IBittyBuzzMessageHandler() = default;

    /**
     *@brief process a message and sends a response if needed. A response is sent to the appropriate
     *queue on a request
     *@return true if the operation was successfull or there was no message to process, false if an
     *reply could not be send (i.e. the queue was full or the message was could not be understood)*/
    virtual bool processMessage() = 0;

    /**@brief clears all the pending messages in the input queue without processing them*/
    virtual void clearMessages() = 0;

    /**
     *@brief gets the number of message to be processed in the queue
     *@return The number of message in the queue
     **/
    virtual uint16_t messageQueueLength() const = 0;
};

#endif // __IBITTYBUZZMESSAGEHANDLER_H_
