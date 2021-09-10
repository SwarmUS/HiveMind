
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

        if (const auto* calibCall = std::get_if<SetInterlocStateDTO>(&(apiCall->getAPICall()))) {
            return handleStateChangeMessage(*calibCall);
        }
    }

    m_logger.log(LogLevel::Warn, "Non supported message in interloc queue");
    return false;
}

// bool InterlocMessageHandler::handleCalibrationMessage(const CalibrationMessageDTO& dto,
//                                                      uint16_t sourceId) const {
//    auto messageVariant = dto.getCall();
//
//    if (const auto* startCalibMsg = std::get_if<StartCalibrationDTO>(&messageVariant)) {
//        if (startCalibMsg->getCalibrationMode() == CalibrationModeDTO::RESPONDER) {
//            m_interlocManager.startCalibSingleResponder(sourceId, notifyCalibrationEndedStatic,
//                                                        (void*)this);
//            return true;
//        }
//
//        if (startCalibMsg->getCalibrationMode() == CalibrationModeDTO::INITIATOR) {
//            m_interlocManager.startCalibSingleInitiator();
//            return true;
//        }
//
//        return false;
//    }
//
//    if (std::holds_alternative<StopCalibrationDTO>(messageVariant)) {
//        m_interlocManager.stopCalibration();
//        return true;
//    }
//
//    if (const auto* setDistanceMsg = std::get_if<SetCalibrationDistanceDTO>(&messageVariant)) {
//        m_interlocManager.configureTWRCalibration(setDistanceMsg->getDistance() * 100);
//        return true;
//    }
//
//    return false;
//}
//
// void InterlocMessageHandler::notifyCalibrationEnded(uint16_t initiatorId) {
//    MessageDTO message = MessageDTO(m_bsp.getUUId(), initiatorId,
//                                    InterlocAPIDTO(CalibrationMessageDTO(CalibrationEndedDTO())));
//
//    if (!getQueueForDestination(initiatorId).push(message)) {
//        m_logger.log(LogLevel::Warn, "Could not push calibration ended message on queue");
//    }
//}

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
        m_interlocManager.configureTWRCalibration(twrConfig->getDistance());
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
        m_logger.log(
            LogLevel::Warn,
            "Interloc Manager changed state without having received a stateChange message");
        return;
    }
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

// void InterlocMessageHandler::notifyCalibrationEndedStatic(void* context, uint16_t
// initiatorId) {
//    static_cast<InterlocMessageHandler*>(context)->notifyCalibrationEnded(initiatorId);
//}
