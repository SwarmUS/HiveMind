#include "BittyBuzzMessageHandler.h"
#include "BittyBuzzSystem.h"
#include "bbzvm.h"
#include <bsp/SettingsContainer.h>

BittyBuzzMessageHandler::BittyBuzzMessageHandler(const IBittyBuzzClosureRegister& closureRegister,
                                                 ICircularQueue<MessageDTO>& inputQueue,
                                                 ICircularQueue<MessageDTO>& hostQueue,
                                                 ICircularQueue<MessageDTO>& remoteQueue,
                                                 const IBSP& bsp,
                                                 ILogger& logger) :
    m_closureRegister(closureRegister),
    m_inputQueue(inputQueue),
    m_hostQueue(hostQueue),
    m_remoteQueue(remoteQueue),
    m_bsp(bsp),
    m_logger(logger) {}

bool BittyBuzzMessageHandler::processMessage() {
    const auto message = m_inputQueue.peek();

    if (message) {
        m_inputQueue.pop();
        const MessageDTO& messageDTO = message.value();
        if (messageDTO.getDestinationId() == m_bsp.getUUId()) {
            return handleMessage(message.value());
        }

        m_logger.log(LogLevel::Warn, "Message destination id does not match");
        return false;
    }

    return true;
}

uint16_t BittyBuzzMessageHandler::messageQueueLength() const { return m_inputQueue.getLength(); }

FunctionListLengthResponseDTO BittyBuzzMessageHandler::handleFunctionListLengthRequest(
    const FunctionListLengthRequestDTO& functionLengthRequest) {
    (void)functionLengthRequest;
    return FunctionListLengthResponseDTO(m_closureRegister.getRegisteredClosureLength());
}

FunctionDescriptionResponseDTO BittyBuzzMessageHandler::handleFunctionDescriptionRequest(

    const FunctionDescriptionRequestDTO& functionDescRequest) {
    uint16_t idx = functionDescRequest.getIndex();
    auto optClosure = m_closureRegister.getRegisteredClosure(idx);

    if (optClosure) {
        const BittyBuzzFunctionDescription& closureDesc = optClosure.value().get().m_description;
        std::array<FunctionDescriptionArgumentDTO, FunctionDescriptionDTO::ARGUMENTS_MAX_SIZE>
            argsDto;

        // Bulding arguments
        auto args = closureDesc.getArguments();
        for (uint16_t i = 0; i < closureDesc.getArgumentsLength(); i++) {
            argsDto[i] = FunctionDescriptionArgumentDTO(std::get<0>(args[i]), std::get<1>(args[i]));
        }

        FunctionDescriptionDTO descDto(closureDesc.getFunctionName(), argsDto.data(),
                                       closureDesc.getArgumentsLength());
        return FunctionDescriptionResponseDTO(descDto);
    }
    return FunctionDescriptionResponseDTO(
        GenericResponseDTO(GenericResponseStatusDTO::BadRequest, "invalid idx"));
}

FunctionCallResponseDTO BittyBuzzMessageHandler::handleFunctionCallRequest(
    const FunctionCallRequestDTO& functionRequest) {

    std::optional<std::reference_wrapper<const BittyBuzzRegisteredClosure>> registeredClosureOpt =
        m_closureRegister.getRegisteredClosure(functionRequest.getFunctionName());

    if (registeredClosureOpt) {
        const BittyBuzzRegisteredClosure& registeredClosure = registeredClosureOpt.value().get();

        const auto& args = functionRequest.getArguments();
        const auto& storedArgs = registeredClosure.m_description.getArguments();
        GenericResponseDTO invalidRequestResponse(GenericResponseStatusDTO::BadRequest, "");

        if (registeredClosure.m_description.getArgumentsLength() !=
            functionRequest.getArgumentsLength()) {
            m_logger.log(LogLevel::Warn, "BBZ: function call request args length don't match");

            invalidRequestResponse.setDetails("arg length don't match");
            return invalidRequestResponse;
        }

        bbzvm_push(registeredClosure.m_selfHeapIdx); // Push self table
        bbzvm_push(registeredClosure.m_closureHeapIdx); // Push closure

        // Pushing bbz args
        for (uint16_t i = 0; i < functionRequest.getArgumentsLength(); i++) {
            // Buzz table index
            const std::variant<std::monostate, int64_t, float>& arg = args[i].getArgument();

            if (const int64_t* intVal = std::get_if<int64_t>(&arg)) {
                // Check if it matches the stored type
                if (std::get<1>(storedArgs[i]) != FunctionDescriptionArgumentTypeDTO::Int) {
                    invalidRequestResponse.setDetails("arg type don't match");
                    return invalidRequestResponse;
                }
                bbzvm_pushi((int16_t)*intVal);

            } else if (const float* floatVal = std::get_if<float>(&arg)) {
                if (std::get<1>(storedArgs[i]) != FunctionDescriptionArgumentTypeDTO::Float) {
                    invalidRequestResponse.setDetails("arg type don't match");
                    return invalidRequestResponse;
                }
                bbzvm_pushf(bbzfloat_fromfloat(*floatVal));
            }
        }

        // Call the closure
        bbzvm_closure_call(functionRequest.getArgumentsLength());
        bbzvm_pop(); // Pop self table

        // response
        return FunctionCallResponseDTO(GenericResponseStatusDTO::Ok, "");
    }
    return FunctionCallResponseDTO(GenericResponseStatusDTO::BadRequest, "Not registred");
}

UserCallResponseDTO BittyBuzzMessageHandler::handleUserCallRequest(
    const UserCallRequestDTO& userRequest) {

    // TODO: handle
    const std::variant<std::monostate, FunctionCallRequestDTO, FunctionListLengthRequestDTO,
                       FunctionDescriptionRequestDTO>& variantReq = userRequest.getRequest();
    if (const auto* fReq = std::get_if<FunctionCallRequestDTO>(&variantReq)) {
        // Response
        FunctionCallResponseDTO fResponse = handleFunctionCallRequest(*fReq);
        return UserCallResponseDTO(UserCallTargetDTO::BUZZ, userRequest.getSource(), fResponse);
    }
    if (const auto* fReq = std::get_if<FunctionDescriptionRequestDTO>(&variantReq)) {
        // Response
        FunctionDescriptionResponseDTO fResponse = handleFunctionDescriptionRequest(*fReq);
        return UserCallResponseDTO(UserCallTargetDTO::BUZZ, userRequest.getSource(), fResponse);
    }
    if (const auto* fReq = std::get_if<FunctionListLengthRequestDTO>(&variantReq)) {
        // Response
        FunctionListLengthResponseDTO fResponse = handleFunctionListLengthRequest(*fReq);
        return UserCallResponseDTO(UserCallTargetDTO::BUZZ, userRequest.getSource(), fResponse);
    }

    return UserCallResponseDTO(
        UserCallTargetDTO::BUZZ, userRequest.getSource(),
        GenericResponseDTO(GenericResponseStatusDTO::BadRequest, "Unknown UC"));
}

ResponseDTO BittyBuzzMessageHandler::handleRequest(const RequestDTO& request) {
    const std::variant<std::monostate, UserCallRequestDTO, HiveMindApiRequestDTO,
                       SwarmApiRequestDTO>& variantReq = request.getRequest();

    if (const auto* uReq = std::get_if<UserCallRequestDTO>(&variantReq)) {
        std::optional<UserCallResponseDTO> uResponse = handleUserCallRequest(*uReq);
        if (uResponse) {
            return ResponseDTO(request.getId(), uResponse.value());
        }
    }

    if (std::holds_alternative<HiveMindApiRequestDTO>(variantReq)) {
        m_logger.log(LogLevel::Warn, "Received Hivemind Req in buzz queue");
    } else if (std::holds_alternative<SwarmApiRequestDTO>(variantReq)) {
        m_logger.log(LogLevel::Warn, "Received Swarm Req in buzz queue");
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
    const std::variant<std::monostate, GenericResponseDTO, UserCallResponseDTO,
                       HiveMindApiResponseDTO, SwarmApiResponseDTO>& variantResp =
        response.getResponse();

    if (const auto* uResp = std::get_if<UserCallResponseDTO>(&variantResp)) {
        return handleUserCallResponse(*uResp);
    }
    if (const auto* gResp = std::get_if<GenericResponseDTO>(&variantResp)) {
        return handleGenericResponse(*gResp);
    }
    if (std::holds_alternative<HiveMindApiResponseDTO>(variantResp)) {
        m_logger.log(LogLevel::Warn, "Received Hivemind Resp in buzz queue");
    } else if (std::holds_alternative<SwarmApiResponseDTO>(variantResp)) {
        m_logger.log(LogLevel::Warn, "Received Swarm Resp in buzz queue");
    }
    return false;
}

bool BittyBuzzMessageHandler::handleMessage(const MessageDTO& message) {
    const std::variant<std::monostate, RequestDTO, ResponseDTO, GreetingDTO>& variantMsg =
        message.getMessage();

    // Handling request
    if (const auto* request = std::get_if<RequestDTO>(&variantMsg)) {
        ResponseDTO response = handleRequest(*request);

        uint16_t uuid = m_bsp.getUUId();
        MessageDTO responseMessage(uuid, message.getSourceId(), response);

        if (responseMessage.getDestinationId() == uuid) {
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
    // should not get greetings here
    if (std::holds_alternative<GreetingDTO>(variantMsg)) {
        m_logger.log(LogLevel::Warn, "Recieved Greeting in buzz queue");
    }

    return false;
}
