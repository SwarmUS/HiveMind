#include "MessageHandlerContainer.h"
#include <bsp/BSPContainer.h>
#include <cpp-common/CircularQueueStack.h>
#include <logger/LoggerContainer.h>

constexpr uint16_t gc_queueMaxSize = 8;

HiveMindApiRequestHandler MessageHandlerContainer::createHiveMindApiRequestHandler() {
    return HiveMindApiRequestHandler(BSPContainer::getBSP(), LoggerContainer::getLogger());
}

MessageDispatcher MessageHandlerContainer::createMessageDispatcher(
    IHiveMindHostDeserializer& deserializer, IHiveMindApiRequestHandler& hivemindApiReqHandler) {
    return MessageDispatcher(getBuzzMsgQueue(), getHostMsgQueue(), getRemoteMsgQueue(),
                             deserializer, hivemindApiReqHandler, BSPContainer::getBSP(),
                             LoggerContainer::getLogger());
}

ThreadSafeQueue<MessageDTO>& MessageHandlerContainer::getBuzzMsgQueue() {
    static CircularQueueStack<MessageDTO, gc_queueMaxSize> s_buzzMsgQueue;
    static ThreadSafeQueue<MessageDTO> s_buzzMsgThreadQueue(s_buzzMsgQueue);

    return s_buzzMsgThreadQueue;
}

ThreadSafeQueue<MessageDTO>& MessageHandlerContainer::getHostMsgQueue() {
    static CircularQueueStack<MessageDTO, gc_queueMaxSize> s_hostMsgQueue;
    static ThreadSafeQueue<MessageDTO> s_hostMsgThreadQueue(s_hostMsgQueue);

    return s_hostMsgThreadQueue;
}

ThreadSafeQueue<MessageDTO>& MessageHandlerContainer::getRemoteMsgQueue() {
    static CircularQueueStack<MessageDTO, gc_queueMaxSize> s_remoteMsgQueue;
    static ThreadSafeQueue<MessageDTO> s_remoteMsgThreadQueue(s_remoteMsgQueue);

    return s_remoteMsgThreadQueue;
}
