
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
            return handleCalibrationMessage(*calibCall);
        }
    }

    m_logger.log(LogLevel::Warn, "Non supported message in interloc queue");
    return false;
}

bool InterlocMessageHandler::handleCalibrationMessage(const CalibrationMessageDTO& dto) {
    auto messageVariant = dto.getCall();

    if (const auto* startCalibMsg = std::get_if<StartCalibrationDTO>(&messageVariant)) {
        (void)startCalibMsg;
        // TODO: Start calib
        return true;
    }

    if (const auto* stopCalibMessage = std::get_if<StopCalibrationDTO>(&messageVariant)) {
        (void)stopCalibMessage;
        // TODO: Stop calib
        return true;
    }

    if (const auto* setDistanceMsg = std::get_if<SetCalibrationDistanceDTO>(&messageVariant)) {
        (void)setDistanceMsg;
        // TODO: Set distance
        return true;
    }

    return false;
}

bool InterlocMessageHandler::notifyCalibrationEnded(uint16_t initiatorId) {
    MessageDTO message = MessageDTO(m_bsp.getUUId(), initiatorId,
                                    InterlocAPIDTO(CalibrationMessageDTO(CalibrationEndedDTO())));

    return getQueueForDestination(initiatorId).push(message);
}

ICircularQueue<MessageDTO>& InterlocMessageHandler::getQueueForDestination(
    uint16_t destinationId) const {
    if (destinationId == m_bsp.getUUId()) {
        return m_hostQueue;
    }

    return m_remoteQueue;
}
