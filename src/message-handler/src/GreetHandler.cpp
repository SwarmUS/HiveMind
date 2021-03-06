#include "GreetHandler.h"

GreetHandler::GreetHandler(IHiveMindHostSerializer& serializer,
                           IHiveMindHostDeserializer& deserializer,
                           IBSP& bsp) :
    m_serializer(serializer), m_deserializer(deserializer), m_bsp(bsp) {}

bool GreetHandler::greet() {
    MessageDTO msg;
    if (m_deserializer.deserializeFromStream(msg)) {
        if (std::holds_alternative<GreetingDTO>(msg.getMessage())) {
            uint16_t uuid = m_bsp.getUUId();
            GreetingDTO greet(uuid);
            MessageDTO message(uuid, uuid, greet);
            return m_serializer.serializeToStream(message);
        }
    }
    return false;
}
