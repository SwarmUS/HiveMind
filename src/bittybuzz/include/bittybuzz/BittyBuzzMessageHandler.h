#ifndef __BITTYBUZZMESSAGEHANDLER_H_
#define __BITTYBUZZMESSAGEHANDLER_H_

#include "IBittyBuzzFunctionRegister.h"
#include <cpp-common/ICircularQueue.h>
#include <hivemind-host/MessageDTO.h>
#include <logger/ILogger.h>

class BittyBuzzMessageHandler {
  public:
    BittyBuzzMessageHandler(const IBittyBuzzFunctionRegister& functionRegister,
                            ICircularQueue<MessageDTO>& inboundQueue,
                            ICircularQueue<MessageDTO>& outBoundQueue,
                            uint16_t bspuuid,
                            ILogger& logger);

    ~BittyBuzzMessageHandler() = default;

    bool processMessage();

  private:
    const IBittyBuzzFunctionRegister& m_functionRegister;
    ICircularQueue<MessageDTO>& m_inboundQueue;
    ICircularQueue<MessageDTO>& m_outboundQueue;
    const uint16_t m_uuid;
    ILogger& m_logger;

    // handling funciton
    FunctionCallResponseDTO handleFunctionCallRequest(
        const FunctionCallRequestDTO& functionRequest);
    UserCallResponseDTO handleUserCallRequest(const UserCallRequestDTO& userRequest);
    ResponseDTO handleRequest(const RequestDTO& request);

    static bool handleUserCallResponse(const UserCallResponseDTO& response);
    static bool handleGenericResponse(const GenericResponseDTO& response);
    static bool handleResponse(const ResponseDTO& response);

    bool handleMessage(const MessageDTO& message);
};

#endif // __BITTYBUZZMESSAGEHANDLER_H_
