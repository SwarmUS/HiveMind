#include "Greeter.h"
#include <hivemind-host/HiveMindHostDeserializer.h>
#include <hivemind-host/HiveMindHostSerializer.h>

Greeter::Greeter(IProtobufStream& stream, IBSP& bsp) : m_stream(stream), m_bsp(bsp) {}

bool Greeter::greet() {
    HiveMindHostDeserializer deserializer(m_stream);

    MessageDTO msg;
    if (deserializer.deserializeFromStream(msg)) {
        if (std::holds_alternative<GreetingDTO>(msg.getMessage())) {
            HiveMindHostSerializer serializer(m_stream);
            uint16_t uuid = m_bsp.getUUId();
            GreetingDTO greet(uuid);
            MessageDTO msg(uuid, uuid, greet);
            return serializer.serializeToStream(msg);
        }
    }
    return false;
}
