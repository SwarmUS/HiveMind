#include "BittyBuzzMessageHandler.h"
#include "BittyBuzzSystem.h"
#include "bbzvm.h"
#include <bsp/SettingsContainer.h>

BittyBuzzMessageHandler::BittyBuzzMessageHandler(const IBittyBuzzClosureRegister& closureRegister,
                                                 ICircularQueue<MessageDTO>& inputQueue,
                                                 ICircularQueue<MessageDTO>& hostQueue,
                                                 ICircularQueue<MessageDTO>& remoteQueue,
                                                 uint16_t bspuuid,
                                                 ILogger& logger) :
    m_closureRegister(closureRegister),
    m_inputQueue(inputQueue),
    m_hostQueue(hostQueue),
    m_remoteQueue(remoteQueue),
    m_uuid(bspuuid),
    m_logger(logger) {}

bool BittyBuzzMessageHandler::processMessage() {
    const auto message = m_inputQueue.peek();

    if (message) {
        m_inputQueue.pop();
        const MessageDTO& messageDTO = message.value();
        if (messageDTO.getDestinationId() == m_uuid) {
            return handleMessage(message.value());
        }

        m_logger.log(LogLevel::Warn, "Message destination id does not match");
        return false;
    }

    return true;
}

uint16_t BittyBuzzMessageHandler::messageQueueLength() const { return m_inputQueue.getLength(); }

FunctionCallResponseDTO BittyBuzzMessageHandler::handleFunctionCallRequest(
    const FunctionCallRequestDTO& functionRequest) {

    std::optional<bbzheap_idx_t> closureHeapIdx =
        m_closureRegister.getClosureHeapIdx(functionRequest.getFunctionName());

    if (closureHeapIdx) {

        const std::array<FunctionCallArgumentDTO,
                         FunctionCallRequestDTO::FUNCTION_CALL_ARGUMENTS_MAX_LENGTH>& args =
            functionRequest.getArguments();

        // Pushing bbz table for arguments
        bbzheap_idx_t table = bbztable_new();

        for (uint16_t i = 0; i < functionRequest.getArgumentsLength(); i++) {
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

        // TODO: Migrate to variable number of arguments once we know the number beforehand (at
        // registration), will avoid the user messing with tables (if possible)
        bbzvm_pushnil(); // Push self table
        bbzvm_push(closureHeapIdx.value());
        bbzvm_push(table);
        bbzvm_closure_call(1);
        bbzvm_pop(); // Pop seldf table

        // response
        return FunctionCallResponseDTO(GenericResponseStatusDTO::Ok, "");
    }
    return FunctionCallResponseDTO(GenericResponseStatusDTO::BadRequest, "");
}

UserCallResponseDTO BittyBuzzMessageHandler::handleUserCallRequest(
    const UserCallRequestDTO& userRequest) {

    const std::variant<std::monostate, FunctionCallRequestDTO>& variantReq =
        userRequest.getRequest();
    if (const auto* fReq = std::get_if<FunctionCallRequestDTO>(&variantReq)) {
        // Response
        FunctionCallResponseDTO fResponse = handleFunctionCallRequest(*fReq);

        return UserCallResponseDTO(UserCallTargetDTO::BUZZ, userRequest.getSource(), fResponse);
    }

    return UserCallResponseDTO(
        UserCallTargetDTO::BUZZ, userRequest.getSource(),
        GenericResponseDTO(GenericResponseStatusDTO::BadRequest, "Unknown UC"));
}

ResponseDTO BittyBuzzMessageHandler::handleRequest(const RequestDTO& request) {
    const std::variant<std::monostate, UserCallRequestDTO>& variantReq = request.getRequest();

    if (const auto* uReq = std::get_if<UserCallRequestDTO>(&variantReq)) {
        std::optional<UserCallResponseDTO> uResponse = handleUserCallRequest(*uReq);
        if (uResponse) {
            return ResponseDTO(request.getId(), uResponse.value());
        }
    }
    return ResponseDTO(request.getId(),
                       GenericResponseDTO(GenericResponseStatusDTO::BadRequest, "Unknown REQ"));
}

bool BittyBuzzMessageHandler::handleUserCallResponse(const UserCallResponseDTO& response) {
    // TODO: handle user call response,
    (void)response;
    return true;
}

bool BittyBuzzMessageHandler::handleGenericResponse(const GenericResponseDTO& response) {
    // TODO: handle generic response
    (void)response;
    return true;
}

bool BittyBuzzMessageHandler::handleResponse(const ResponseDTO& response) {
    const std::variant<std::monostate, GenericResponseDTO, UserCallResponseDTO>& variantResp =
        response.getResponse();

    if (const auto* uResp = std::get_if<UserCallResponseDTO>(&variantResp)) {
        return handleUserCallResponse(*uResp);
    }
    if (const auto* gResp = std::get_if<GenericResponseDTO>(&variantResp)) {
        return handleGenericResponse(*gResp);
    }
    return false;
}

bool BittyBuzzMessageHandler::handleMessage(const MessageDTO& message) {
    const std::variant<std::monostate, RequestDTO, ResponseDTO>& variantMsg = message.getMessage();

    // Handling request
    if (const auto* request = std::get_if<RequestDTO>(&variantMsg)) {
        ResponseDTO response = handleRequest(*request);

        MessageDTO responseMessage(m_uuid, message.getSourceId(), response);

        if (responseMessage.getDestinationId() == m_uuid) {
            // Sending response to host
            return m_hostQueue.push(responseMessage);
        }

        // Sending response to remote
        return m_remoteQueue.push(responseMessage);
    }

    // Handle response
    if (const auto* response = std::get_if<ResponseDTO>(&variantMsg)) {
        return handleResponse(*response);
    }

    return false;
}