
#include <cpp-common/ICircularQueue.h>
#include <interloc/InterlocMessageHandler.h>

InterlocMessageHandler::InterlocMessageHandler(ILogger& logger,
                                               IInterlocManager& interlocManager,
                                               IBSP& bsp,
                                               ICircularQueue<MessageDTO>& inputQueue) :
    m_logger(logger), m_interlocManager(interlocManager), m_bsp(bsp), m_inputQueue(inputQueue) {}

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
