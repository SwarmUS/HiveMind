
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
    m_remoteQueue(remoteQueue) {}

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

bool InterlocMessageHandler::handleMessage(const MessageDTO& dto) const {
    auto message = dto.getMessage();

    if (const auto* apiCall = std::get_if<InterlocAPIDTO>(&message)) {
        if (const auto* calibCall = std::get_if<CalibrationMessageDTO>(&(apiCall->getAPICall()))) {
            return handleCalibrationMessage(*calibCall, dto.getSourceId());
        }
    }

    m_logger.log(LogLevel::Warn, "Non supported message in interloc queue");
    return false;
}

bool InterlocMessageHandler::handleCalibrationMessage(const CalibrationMessageDTO& dto,
                                                      uint16_t sourceId) const {
    auto messageVariant = dto.getCall();

    if (const auto* startCalibMsg = std::get_if<StartCalibrationDTO>(&messageVariant)) {
        if (startCalibMsg->getCalibrationMode() == CalibrationModeDTO::RESPONDER) {
            m_interlocManager.startCalibSingleResponder(sourceId, notifyCalibrationEndedStatic,
                                                        (void*)this);
            return true;
        }

        if (startCalibMsg->getCalibrationMode() == CalibrationModeDTO::INITIATOR) {
            m_interlocManager.startCalibSingleInitiator();
            return true;
        }

        return false;
    }

    if (std::holds_alternative<StopCalibrationDTO>(messageVariant)) {
        m_interlocManager.stopCalibration();
        return true;
    }

    if (const auto* setDistanceMsg = std::get_if<SetCalibrationDistanceDTO>(&messageVariant)) {
        m_interlocManager.setCalibDistance(setDistanceMsg->getDistance() * 100);
        return true;
    }

    return false;
}

void InterlocMessageHandler::notifyCalibrationEnded(uint16_t initiatorId) {
    MessageDTO message = MessageDTO(m_bsp.getUUId(), initiatorId,
                                    InterlocAPIDTO(CalibrationMessageDTO(CalibrationEndedDTO())));

    if (!getQueueForDestination(initiatorId).push(message)) {
        m_logger.log(LogLevel::Warn, "Could not push calibration ended message on queue");
    }
}

ICircularQueue<MessageDTO>& InterlocMessageHandler::getQueueForDestination(
    uint16_t destinationId) const {
    if (destinationId == m_bsp.getUUId()) {
        return m_hostQueue;
    }

    return m_remoteQueue;
}

void InterlocMessageHandler::notifyCalibrationEndedStatic(void* context, uint16_t initiatorId) {
    static_cast<InterlocMessageHandler*>(context)->notifyCalibrationEnded(initiatorId);
}
