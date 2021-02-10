#include "MessageSender.h"

MessageSender::MessageSender(ICircularQueue<MessageDTO>& inputQueue,
                             IHiveMindHostSerializer& serializer,
                             ILogger& logger) :
    m_inputQueue(inputQueue), m_serializer(serializer), m_logger(logger) {}

bool MessageSender::processAndSerialize() {
    const std::optional<std::reference_wrapper<const MessageDTO>> message = m_inputQueue.peek();
    if (message) {

        return m_serializer.serializeToStream(message.value());
    }

    return false;
}
