#ifndef __BITTYBUZZMESSAGEHANDLER_H_
#define __BITTYBUZZMESSAGEHANDLER_H_

#include "IBittyBuzzFunctionRegister.h"
#include "IBittyBuzzMessageHandler.h"
#include <cpp-common/ICircularQueue.h>
#include <hivemind-host/MessageDTO.h>
#include <logger/ILogger.h>

class BittyBuzzMessageHandler : public IBittyBuzzMessageHandler {
  public:
    BittyBuzzMessageHandler(const IBittyBuzzFunctionRegister& functionRegister,
                            ICircularQueue<MessageDTO>& inputQueue,
                            ICircularQueue<MessageDTO>& hostQueue,
                            ICircularQueue<MessageDTO>& remoteQueue,
                            uint16_t bspuuid,
                            ILogger& logger);

    ~BittyBuzzMessageHandler() override = default;

    bool processMessage() override;

    uint16_t messageQueueLength() const override;

  private:
    const IBittyBuzzFunctionRegister& m_functionRegister;
    ICircularQueue<MessageDTO>& m_inputQueue;
    ICircularQueue<MessageDTO>& m_hostQueue;
    ICircularQueue<MessageDTO>& m_remoteQueue;
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
