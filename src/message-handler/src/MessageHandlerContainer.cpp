#include "MessageHandlerContainer.h"
#include "IGreetHandler.h"
#include <InterlocSettings.h>
#include <Mutex.h>
#include <bsp/BSPContainer.h>
#include <cpp-common/CircularQueueStack.h>
#include <logger/LoggerContainer.h>

constexpr uint16_t gc_queueMaxSize = 8;
constexpr uint16_t gc_interlocPosUpdateMaxSize = 4 * MAX_ROBOTS_IN_SWARM;

HiveMindHostApiRequestHandler MessageHandlerContainer::createHiveMindHostApiRequestHandler() {
    return HiveMindHostApiRequestHandler(BSPContainer::getBSP(), getHostMsgQueue(),
                                         LoggerContainer::getLogger());
}

MessageDispatcher MessageHandlerContainer::createMessageDispatcher(
    IHiveMindHostDeserializer& deserializer,
    IHiveMindHostApiRequestHandler& hivemindApiReqHandler,
    IGreetSender& greetSender) {
    return MessageDispatcher(getBuzzMsgQueue(), getHostMsgQueue(), getRemoteMsgQueue(),
                             getInterlocMsgQueue(), deserializer, hivemindApiReqHandler,
                             greetSender, BSPContainer::getBSP(), LoggerContainer::getLogger());
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

ThreadSafeQueue<MessageDTO>& MessageHandlerContainer::getInterlocMsgQueue() {
    static Mutex s_mutex(10);
    static CircularQueueStack<MessageDTO, gc_queueMaxSize> s_interlocMsgQueue;
    static ThreadSafeQueue<MessageDTO> s_interlocMsgThreadQueue(s_interlocMsgQueue, s_mutex);

    return s_interlocMsgThreadQueue;
}

ThreadSafeQueue<uint16_t>& MessageHandlerContainer::getInterlocPosUpdateQueue() {
    static Mutex s_mutex(10);
    static CircularQueueStack<uint16_t, gc_interlocPosUpdateMaxSize> s_interlocMsgQueue;
    static ThreadSafeQueue<uint16_t> s_interlocPosUpdateThreadQueue(s_interlocMsgQueue, s_mutex);

    return s_interlocPosUpdateThreadQueue;
}
