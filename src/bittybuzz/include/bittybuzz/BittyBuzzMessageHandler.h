#ifndef __BITTYBUZZMESSAGEHANDLER_H_
#define __BITTYBUZZMESSAGEHANDLER_H_

#include "IBittyBuzzClosureRegister.h"
#include "IBittyBuzzMessageHandler.h"
#include <bsp/IBSP.h>
#include <cpp-common/ICircularQueue.h>
#include <hivemind-host/MessageDTO.h>
#include <logger/ILogger.h>

class BittyBuzzMessageHandler : public IBittyBuzzMessageHandler {
  public:
    BittyBuzzMessageHandler(const IBittyBuzzClosureRegister& closureRegister,
                            ICircularQueue<MessageDTO>& inputQueue,
                            ICircularQueue<MessageDTO>& hostQueue,
                            ICircularQueue<MessageDTO>& remoteQueue,
                            const IBSP& bsp,
                            ILogger& logger);

    ~BittyBuzzMessageHandler() override = default;

    bool processMessage() override;

    uint16_t messageQueueLength() const override;

  private:
    const IBittyBuzzClosureRegister& m_closureRegister;
    ICircularQueue<MessageDTO>& m_inputQueue;
    ICircularQueue<MessageDTO>& m_hostQueue;
    ICircularQueue<MessageDTO>& m_remoteQueue;
    const IBSP& m_bsp;
    ILogger& m_logger;

    // handling funciton
    FunctionListLengthResponseDTO handleFunctionListLengthRequest(
        const FunctionListLengthRequestDTO& functionLengthRequest);
    FunctionDescriptionResponseDTO handleFunctionDescriptionRequest(
        const FunctionDescriptionRequestDTO& functionDescRequest);
    FunctionCallResponseDTO handleFunctionCallRequest(
        const FunctionCallRequestDTO& functionRequest);
    UserCallResponseDTO handleUserCallRequest(const UserCallRequestDTO& userRequest);
    ResponseDTO handleRequest(const RequestDTO& request);

    static bool handleUserCallResponse(const UserCallResponseDTO& response);
    static bool handleGenericResponse(const GenericResponseDTO& response);
    bool handleResponse(const ResponseDTO& response);

    bool handleMessage(const MessageDTO& message);
};

#endif // __BITTYBUZZMESSAGEHANDLER_H_
