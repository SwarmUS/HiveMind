#include "MessageHandlerContainer.h"
#include "IGreetHandler.h"
#include <Mutex.h>
#include <bsp/BSPContainer.h>
#include <cpp-common/CircularQueueStack.h>
#include <logger/LoggerContainer.h>

constexpr uint16_t gc_queueMaxSize = 8;

HiveMindApiRequestHandler MessageHandlerContainer::createHiveMindApiRequestHandler() {
    return HiveMindApiRequestHandler(BSPContainer::getBSP(), LoggerContainer::getLogger());
}

MessageDispatcher MessageHandlerContainer::createMessageDispatcher(
    IHiveMindHostDeserializer& deserializer,
    IHiveMindApiRequestHandler& hivemindApiReqHandler,
    IGreetSender& greetSender) {
    return MessageDispatcher(getBuzzMsgQueue(), getHostMsgQueue(), getRemoteMsgQueue(),
                             deserializer, hivemindApiReqHandler, greetSender,
                             BSPContainer::getBSP(), LoggerContainer::getLogger());
}

ThreadSafeQueue<MessageDTO>& MessageHandlerContainer::getBuzzMsgQueue() {
    static Mutex s_mutex(10);
    static CircularQueueStack<MessageDTO, gc_queueMaxSize> s_buzzMsgQueue;
    static ThreadSafeQueue<MessageDTO> s_buzzMsgThreadQueue(s_buzzMsgQueue, s_mutex);

    return s_buzzMsgThreadQueue;
}

ThreadSafeQueue<MessageDTO>& MessageHandlerContainer::getHostMsgQueue() {
    static Mutex s_mutex(10);
    static CircularQueueStack<MessageDTO, gc_queueMaxSize> s_hostMsgQueue;
    static ThreadSafeQueue<MessageDTO> s_hostMsgThreadQueue(s_hostMsgQueue, s_mutex);

    return s_hostMsgThreadQueue;
}

ThreadSafeQueue<MessageDTO>& MessageHandlerContainer::getRemoteMsgQueue() {
    static Mutex s_mutex(10);
    static CircularQueueStack<MessageDTO, gc_queueMaxSize> s_remoteMsgQueue;
    static ThreadSafeQueue<MessageDTO> s_remoteMsgThreadQueue(s_remoteMsgQueue, s_mutex);

    return s_remoteMsgThreadQueue;
}
