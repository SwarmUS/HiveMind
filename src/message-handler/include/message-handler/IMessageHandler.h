#ifndef __IMESSAGEHANDLER_H_
#define __IMESSAGEHANDLER_H_

#include <hivemind-host/MessageDTO.h>

class IMessageHandler {
  public:
    virtual ~IMessageHandler() = default;

    virtual bool handleMessage(const MessageDTO& message) = 0;
};

#endif // __IMESSAGEHANDLER_H_
