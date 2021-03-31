#include "GreetSender.h"

GreetSender::GreetSender(ICircularQueue<MessageDTO>& outputQueue, IBSP& bsp) :
    m_outputQueue(outputQueue), m_bsp(bsp) {}

bool GreetSender::sendGreet() {
    uint16_t uuid = m_bsp.getUUId();
    GreetingDTO greet(uuid);
    MessageDTO msg(uuid, uuid, greet);
    return m_outputQueue.push(msg);
}
