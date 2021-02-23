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
    FunctionCallResponseDTO handleFunctionCallRequest(
        const FunctionCallRequestDTO& functionRequest);
    std::optional<UserCallResponseDTO> handleUserCallRequest(const UserCallRequestDTO& userRequest);
    std::optional<ResponseDTO> handleRequest(const RequestDTO& request);

    bool handleUserCallResponse(const MessageDTO& message, const UserCallResponseDTO& response);
    bool handleResponse(const MessageDTO& message, const ResponseDTO& response);

    bool handleMessage(const MessageDTO& message);
};

#endif // __BITTYBUZZMESSAGEHANDLER_H_
