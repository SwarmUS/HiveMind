
#include <Task.h>
#include <cpp-common/ICircularQueue.h>
#include <interloc/InterlocMessageHandler.h>

InterlocMessageHandler::InterlocMessageHandler(ILogger& logger,
                                               IInterlocManager& interlocManager,
                                               IBSP& bsp,
                                               ICircularQueue<MessageDTO>& inputQueue,
                                               ICircularQueue<MessageDTO>& hostQueue,
                                               ICircularQueue<MessageDTO>& remoteQueue) :
    m_logger(logger),
    m_interlocManager(interlocManager),
    m_bsp(bsp),
    m_inputQueue(inputQueue),
    m_hostQueue(hostQueue),
    m_remoteQueue(remoteQueue),
    m_messageSourceId(0) {
    m_interlocManager.setInterlocManagerRawAngleDataCallback(rawAngleDataCallbackStatic, this);
    m_interlocManager.setInterlocManagerStateChangeCallback(stateChangeCallbackStatic, this);
}

bool InterlocMessageHandler::processMessage() {
    auto message = m_inputQueue.peek();

    if (message) {
        m_inputQueue.pop();

        if (message.value().get().getDestinationId() != m_bsp.getUUId()) {
            m_logger.log(LogLevel::Warn,
                         "Message with incorrect destination in interloc input queue");
            return false;
        }

        return handleMessage(message.value());
    }

    return true;
}

bool InterlocMessageHandler::handleMessage(const MessageDTO& dto) {
    auto message = dto.getMessage();

    if (const auto* apiCall = std::get_if<InterlocAPIDTO>(&message)) {
        m_messageSourceId = dto.getSourceId();

        if (const auto* setState = std::get_if<SetInterlocStateDTO>(&(apiCall->getAPICall()))) {
            return handleStateChangeMessage(*setState);
        }

        if (const auto* configure =
                std::get_if<InterlocConfigurationDTO>(&(apiCall->getAPICall()))) {
            return handleConfigurationMessage(*configure);
        }
    }

    m_logger.log(LogLevel::Warn, "Non supported message in interloc queue");
    return false;
}

ICircularQueue<MessageDTO>& InterlocMessageHandler::getQueueForDestination(
    uint16_t destinationId) const {
    if (destinationId == m_bsp.getUUId()) {
        return m_hostQueue;
    }

    return m_remoteQueue;
}
bool InterlocMessageHandler::handleStateChangeMessage(const SetInterlocStateDTO dto) const {
    m_interlocManager.setInterlocManagerState(dto.getState());
    return true;
}

bool InterlocMessageHandler::handleConfigurationMessage(const InterlocConfigurationDTO& dto) const {
    auto messageVariant = dto.getConfigurationMessage();

    if (const auto* angleConfig = std::get_if<ConfigureAngleCalibrationDTO>(&messageVariant)) {
        m_interlocManager.configureAngleCalibration(angleConfig->getNumberOfFrames());
        return true;
    }

    if (const auto* twrConfig = std::get_if<ConfigureTWRCalibrationDTO>(&messageVariant)) {
        m_interlocManager.configureTWRCalibration(twrConfig->getDistance() * 100);
        return true;
    }

    return false;
}

void InterlocMessageHandler::stateChangeCallback(InterlocStateDTO previousState,
                                                 InterlocStateDTO newState) {
    if (m_messageSourceId == 0) {
        m_logger.log(
            LogLevel::Warn,
            "Interloc Manager changed state without having received a stateChange message");
        return;
    }

    MessageDTO msg = MessageDTO(
        m_bsp.getUUId(), m_messageSourceId,
        InterlocAPIDTO(InterlocOutputMessageDTO(InterlocStateChangeDTO(previousState, newState))));

    if (!getQueueForDestination(m_messageSourceId).push(msg)) {
        m_logger.log(LogLevel::Warn, "Could not push InterlocStateChange message in queue");
    }
}

void InterlocMessageHandler::rawAngleDataCallback(BspInterlocRawAngleData& data) {
    if (m_messageSourceId == 0) {
        m_logger.log(LogLevel::Warn,
                     "Interloc Manager sending raw data without having received request");
        return;
    }

    uint8_t maxFramesPerMessage = InterlocRawAngleDataDTO::INTERLOC_RAW_ANGLE_FRAMES_MAX_SIZE;

    for (uint8_t i = 0; i < data.m_framesLength / maxFramesPerMessage; i++) {

        MessageDTO msg = MessageDTO(m_bsp.getUUId(), m_messageSourceId,
                                    InterlocAPIDTO(InterlocOutputMessageDTO(constructRawDataMessage(
                                        data, i * maxFramesPerMessage, maxFramesPerMessage))));

        ensureSendMessage(msg);
    }

    uint8_t remaining = data.m_framesLength % maxFramesPerMessage;
    MessageDTO msg = MessageDTO(m_bsp.getUUId(), m_messageSourceId,
                                InterlocAPIDTO(InterlocOutputMessageDTO(constructRawDataMessage(
                                    data, data.m_framesLength - remaining, remaining))));

    ensureSendMessage(msg);
}

void InterlocMessageHandler::ensureSendMessage(MessageDTO& msg) {
    // TODO: add timeout so we don't block forever
    while (getQueueForDestination(m_messageSourceId).isFull()) {
        Task::delay(10);
    }
    getQueueForDestination(m_messageSourceId).push(msg);
}

InterlocRawAngleDataDTO InterlocMessageHandler::constructRawDataMessage(
    BspInterlocRawAngleData& data, uint32_t fromId, uint8_t numFrames) {
    InterlocRxFrameInfoDTO
        frameInfos[InterlocRxFrameRawAngleDataDTO::INTERLOC_BEEBOARDS_SIZE_MAX_LENGTH];
    InterlocRxFrameRawAngleDataDTO
        frames[InterlocRawAngleDataDTO::INTERLOC_RAW_ANGLE_FRAMES_MAX_SIZE];
    InterlocRawAngleDataDTO dataDto;

    for (uint8_t j = 0; j < numFrames; j++) {
        uint32_t frameId = fromId + j;

        for (uint8_t k = 0; k < data.m_frames[frameId].m_frameInfosLength; k++) {
            frameInfos[k].setBeeboardPort(data.m_frames[frameId].m_frameInfos[k].m_beeboardPort);
            frameInfos[k].setRxTimestamp(data.m_frames[frameId].m_frameInfos[k].m_rxTimestamp);
            frameInfos[k].setSfdAngle(data.m_frames[frameId].m_frameInfos[k].m_sfdAngle);
            frameInfos[k].setAccumulatorAngle(
                data.m_frames[frameId].m_frameInfos[k].m_accumulatorAngle);
        }

        frames[j].setFrameInfos(frameInfos, data.m_frames[frameId].m_frameInfosLength);
        frames[j].setFrameId(frameId);
    }

    dataDto.setFrames(frames, numFrames);
    return dataDto;
}

void InterlocMessageHandler::rawAngleDataCallbackStatic(void* context,
                                                        BspInterlocRawAngleData& data) {
    static_cast<InterlocMessageHandler*>(context)->rawAngleDataCallback(data);
}

void InterlocMessageHandler::stateChangeCallbackStatic(void* context,
                                                       InterlocStateDTO previousState,
                                                       InterlocStateDTO newState) {
    static_cast<InterlocMessageHandler*>(context)->stateChangeCallback(previousState, newState);
}