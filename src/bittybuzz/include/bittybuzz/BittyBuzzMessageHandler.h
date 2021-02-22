#ifndef __BITTYBUZZMESSAGEHANDLER_H_
#define __BITTYBUZZMESSAGEHANDLER_H_

#include "IBittyBuzzFunctionRegister.h"
#include <cpp-common/ICircularQueue.h>
#include <hivemind-host/MessageDTO.h>

class BittyBuzzMessageHandler {
  public:
    BittyBuzzMessageHandler(const IBittyBuzzFunctionRegister& functionRegister,
                            ICircularQueue<MessageDTO>& inboundQueue,
                            ICircularQueue<MessageDTO>& outBoundQueue);

    ~BittyBuzzMessageHandler() = default;

    bool processMessage();

  private:
    const IBittyBuzzFunctionRegister& m_functionRegister;
    ICircularQueue<MessageDTO>& m_inboundQueue;
    ICircularQueue<MessageDTO>& m_outboundQueue;

    // handling funciton
    bool handleFunctionCallRequest(const MessageDTO& message,
                                   const FunctionCallRequestDTO& request);
    bool handleUserCallRequest(const MessageDTO& message, const UserCallRequestDTO& request);
    bool handleUserCallResponse(const MessageDTO& message, const UserCallResponseDTO& response);
    bool handleRequest(const MessageDTO& message, const RequestDTO& request);
    bool handleResponse(const MessageDTO& message, const ResponseDTO& response);
    bool handleMessage(const MessageDTO& message);
};

#endif // __BITTYBUZZMESSAGEHANDLER_H_
