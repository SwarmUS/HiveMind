#include "BittyBuzzMessageHandler.h"
#include "BittyBuzzSystem.h"
#include "bbzvm.h"
#include <bbzinmsg.h>
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

        uint32_t destinationId = messageDTO.getDestinationId();

        if (destinationId == 0 || destinationId == m_bsp.getUUId()) {
            return handleMessage(message.value());
        }

        m_logger.log(LogLevel::Warn, "Message destination id does not match");
        return false;
    }

    return true;
}

void BittyBuzzMessageHandler::clearMessages() { m_inputQueue.clear(); }

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

        uint16_t expectedLength = registeredClosure.m_description.getArgumentsLength();
        uint16_t requiredLength = functionRequest.getArgumentsLength();
        if (expectedLength != requiredLength) {
            m_logger.log(
                LogLevel::Warn,
                "BBZ: function call request args length don't match. Expect :%d, received %d",
                expectedLength, requiredLength);

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
    return FunctionCallResponseDTO(GenericResponseStatusDTO::BadRequest, "Not registered");
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
    const std::variant<std::monostate, UserCallRequestDTO, HiveMindHostApiRequestDTO>& variantReq =
        request.getRequest();

    if (const auto* uReq = std::get_if<UserCallRequestDTO>(&variantReq)) {
        std::optional<UserCallResponseDTO> uResponse = handleUserCallRequest(*uReq);
        if (uResponse) {
            return ResponseDTO(request.getId(), uResponse.value());
        }
    }

    m_logger.log(LogLevel::Warn, "Unkown request in buzz queue, idx: %d", variantReq.index());
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

bool BittyBuzzMessageHandler::handleVmMessage(const VmMessageDTO& vmMsg) {

    const std::variant<std::monostate, BuzzMessagesDTO>& msg = vmMsg.getMessage();

    if (const auto* buzzMsg = std::get_if<BuzzMessagesDTO>(&msg)) {
        return handleBuzzMessages(*buzzMsg);
    }

    return false;
}

bool BittyBuzzMessageHandler::handleBuzzMessages(const BuzzMessagesDTO& msg) {

    auto msgArray = msg.getMessages();
    for (uint8_t i = 0; i < msg.getMessagesLength(); i++) {
        bbzmsg_payload_t payload;
        payload.buffer = const_cast<uint8_t*>(msgArray[i].getPayload().data());
        payload.datastart = 0;
        payload.elsize = 1;
        payload.dataend = msgArray[i].getPayloadLength();
        payload.capacity = msgArray[i].getPayloadLength();

        if (bbzinmsg_queue_size() > BBZINMSG_QUEUE_CAP) {
            m_logger.log(LogLevel::Warn, "Overwriting buzz message");
        }

        bbzinmsg_queue_append(&payload);
    }
    return true;
}

bool BittyBuzzMessageHandler::handleResponse(const ResponseDTO& response) {
    const std::variant<std::monostate, GenericResponseDTO, UserCallResponseDTO,
                       HiveMindHostApiResponseDTO>& variantResp = response.getResponse();

    if (const auto* uResp = std::get_if<UserCallResponseDTO>(&variantResp)) {
        return handleUserCallResponse(*uResp);
    }
    if (const auto* gResp = std::get_if<GenericResponseDTO>(&variantResp)) {
        return handleGenericResponse(*gResp);
    }
    if (std::holds_alternative<HiveMindHostApiResponseDTO>(variantResp)) {
        m_logger.log(LogLevel::Warn, "Received Hivemind Resp in buzz queue");
    } else {
        m_logger.log(LogLevel::Warn, "Unkown Resp in buzz queue, idx: %d", variantResp.index());
    }
    return false;
}

bool isBuzzToBuzzUserCallRequest(const RequestDTO& request) {
    if (const auto* uReq = std::get_if<UserCallRequestDTO>(&request.getRequest())) {
        return uReq->getDestination() == UserCallTargetDTO::BUZZ &&
               uReq->getSource() == UserCallTargetDTO::BUZZ;
    }
    return false;
}

bool BittyBuzzMessageHandler::handleMessage(const MessageDTO& message) {
    const std::variant<std::monostate, RequestDTO, ResponseDTO, GreetingDTO, VmMessageDTO,
                       NetworkApiDTO, InterlocAPIDTO, HiveConnectHiveMindApiDTO>& variantMsg =
        message.getMessage();

    // Handling request
    if (const auto* request = std::get_if<RequestDTO>(&variantMsg)) {
        ResponseDTO response = handleRequest(*request);

        uint16_t uuid = m_bsp.getUUId();
        MessageDTO responseMessage(uuid, message.getSourceId(), response);

        if (responseMessage.getDestinationId() == uuid &&
            isBuzzToBuzzUserCallRequest(
                *request)) { // Check it's from the bbvm hosted on this device
            return m_inputQueue.push(responseMessage); // handle it on the next loop
        }
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

    // Handle response
    if (const auto* vmMsg = std::get_if<VmMessageDTO>(&variantMsg)) {
        return handleVmMessage(*vmMsg);
    }

    m_logger.log(LogLevel::Warn, "Recieved unkown message in buzz queue, idx: %d",
                 variantMsg.index());
    return false;
}
