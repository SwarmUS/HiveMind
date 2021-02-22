#include "BittyBuzzMessageHandler.h"

BittyBuzzMessageHandler::BittyBuzzMessageHandler(const IBittyBuzzFunctionRegister& functionRegister,
                                                 ICircularQueue<MessageDTO>& inboundQueue,
                                                 ICircularQueue<MessageDTO>& outBoundQueue) :
    m_functionRegister(functionRegister),
    m_inboundQueue(inboundQueue),
    m_outboundQueue(outBoundQueue) {}

bool BittyBuzzMessageHandler::processMessage() { return true; }

bool BittyBuzzMessageHandler::handleFunctionCallRequest(const MessageDTO& message,
                                                        const FunctionCallRequestDTO& request) {}

bool BittyBuzzMessageHandler::handleUserCallRequest(const MessageDTO& message,
                                                    const UserCallRequestDTO& request) {
    const std::variant<std::monostate, FunctionCallRequestDTO>& variantReq = request.getRequest();

    if (const FunctionCallRequestDTO* fReq = std::get_if<FunctionCallRequestDTO>(&variantReq)) {
        return handleFunctionCallRequest(message, *fReq);
    }
    return false;
}

bool BittyBuzzMessageHandler::handleUserCallResponse(const MessageDTO& message,
                                                     const UserCallResponseDTO& response) {}

bool BittyBuzzMessageHandler::handleRequest(const MessageDTO& message, const RequestDTO& request) {
    const std::variant<std::monostate, UserCallRequestDTO>& variantReq = request.getRequest();

    if (const UserCallRequestDTO* uReq = std::get_if<UserCallRequestDTO>(&variantReq)) {
        return handleUserCallRequest(message, *uReq);
    }
    return false;
}

bool BittyBuzzMessageHandler::handleResponse(const MessageDTO& message,
                                             const ResponseDTO& response) {
    const std::variant<std::monostate, UserCallResponseDTO>& variantResp = response.getResponse();

    if (const UserCallResponseDTO* uResp = std::get_if<UserCallResponseDTO>(&variantResp)) {
        return handleUserCallResponse(message, *uResp);
    }
    return false;
}

bool BittyBuzzMessageHandler::handleMessage(const MessageDTO& message) {
    const std::variant<std::monostate, RequestDTO, ResponseDTO>& variantMsg = message.getMessage();
    if (const RequestDTO* request = std::get_if<RequestDTO>(&variantMsg)) {
        return handleRequest(message, *request);
    }
    if (const ResponseDTO* response = std::get_if<ResponseDTO>(&variantMsg)) {
        return handleResponse(message, *response);
    }
    return false;
}
