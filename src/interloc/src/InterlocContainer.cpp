#include <bsp/BSPContainer.h>
#include <cpp-common/CircularQueue.h>
#include <interloc/Interloc.h>
#include <interloc/InterlocContainer.h>
#include <interloc/InterlocMessageHandler.h>
#include <logger/LoggerContainer.h>
#include <message-handler/MessageHandlerContainer.h>

IInterloc& InterlocContainer::getInterloc() {
    static Interloc s_interloc =
        Interloc(LoggerContainer::getLogger(), BSPContainer::getInterlocManager());
    return s_interloc;
}

IInterlocMessageHandler& InterlocContainer::getInterlocMessageHandler() {

    static InterlocMessageHandler s_messageHandler = InterlocMessageHandler(
        LoggerContainer::getLogger(), BSPContainer::getInterlocManager(), BSPContainer::getBSP(),
        MessageHandlerContainer::getInterlocMsgQueue(), MessageHandlerContainer::getHostMsgQueue(),
        MessageHandlerContainer::getRemoteMsgQueue());

    return s_messageHandler;
}

ThreadSafeQueue<uint16_t>& InterlocContainer::getInterlocPosUpdateQueue() {
    static Mutex s_mutex(10);
    static CircularQueueStack<uint16_t, gc_interlocPosUpdateMaxSize> s_interlocMsgQueue;
    static ThreadSafeQueue<uint16_t> s_interlocPosUpdateThreadQueue(s_interlocMsgQueue, s_mutex);

    return s_interlocPosUpdateThreadQueue;
}
