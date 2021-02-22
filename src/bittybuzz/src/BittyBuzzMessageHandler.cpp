#include "BittyBuzzMessageHandler.h"
#include "BittyBuzzSystem.h"
#include "bbzvm.h"

BittyBuzzMessageHandler::BittyBuzzMessageHandler(const IBittyBuzzFunctionRegister& functionRegister,
                                                 ICircularQueue<MessageDTO>& inboundQueue,
                                                 ICircularQueue<MessageDTO>& outBoundQueue) :
    m_functionRegister(functionRegister),
    m_inboundQueue(inboundQueue),
    m_outboundQueue(outBoundQueue) {}

bool BittyBuzzMessageHandler::processMessage() { return true; }

bool BittyBuzzMessageHandler::handleFunctionCallRequest(const MessageDTO& message,
                                                        const FunctionCallRequestDTO& request) {

    std::optional<uint16_t> functionId =
        m_functionRegister.getFunctionId(request.getFunctionName());

    if (functionId) {

        const std::array<FunctionCallArgumentDTO,
                         FunctionCallRequestDTO::FUNCTION_CALL_ARGUMENTS_MAX_LENGTH>& args =
            request.getArguments();

        // Pushing bbz table for arguments
        bbzheap_idx_t table = bbztable_new();

        for (uint16_t i = 0; i < request.getArgumentsLength(); i++) {
            // Buzz table index
            bbzheap_idx_t tableIdx = bbzint_new((int16_t)i);

            const std::variant<std::monostate, int64_t, float>& arg = args[i].getArgument();

            if (const int64_t* intVal = std::get_if<int64_t>(&arg)) {
                bbzheap_idx_t bbzIntVal = bbzint_new(*intVal);
                bbztable_set(table, tableIdx, bbzIntVal);
            }

            else if (const float* floatVal = std::get_if<float>(&arg)) {
                bbzheap_idx_t bbzFloatVal = bbzfloat_new(*floatVal);
                bbztable_set(table, tableIdx, bbzFloatVal);
            }
        }

        bbzvm_push(table);
        BittyBuzzSystem::functionCall(functionId.value());

        // Send response
        return true;
    }

    return false;
}

bool BittyBuzzMessageHandler::handleUserCallRequest(const MessageDTO& message,
                                                    const UserCallRequestDTO& request) {
    const std::variant<std::monostate, FunctionCallRequestDTO>& variantReq = request.getRequest();

    if (const FunctionCallRequestDTO* fReq = std::get_if<FunctionCallRequestDTO>(&variantReq)) {
        return handleFunctionCallRequest(message, *fReq);
    }
    return false;
}

bool BittyBuzzMessageHandler::handleUserCallResponse(const MessageDTO& message,
                                                     const UserCallResponseDTO& response) {
    (void)message;
    (void)response;
    return true;
}

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
