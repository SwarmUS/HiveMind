#include <InterlocSettings.h>
#include <bsp/BSPContainer.h>
#include <cpp-common/CircularQueueStack.h>
#include <interloc/Interloc.h>
#include <interloc/InterlocContainer.h>
#include <interloc/InterlocMessageHandler.h>
#include <logger/LoggerContainer.h>
#include <message-handler/MessageHandlerContainer.h>

constexpr uint16_t gc_interlocPosUpdateMaxSize = 4 * MAX_ROBOTS_IN_SWARM;

IInterloc& InterlocContainer::getInterloc() {
    static Interloc s_interloc = Interloc(
        LoggerContainer::getLogger(), BSPContainer::getInterlocManager(),
        getInterlocMessageHandler(), getInterlocUpdateOutputQueue(), getInterlocUpdateInputQueue());
    return s_interloc;
}

IInterlocMessageHandler& InterlocContainer::getInterlocMessageHandler() {

    static InterlocMessageHandler s_messageHandler = InterlocMessageHandler(
        LoggerContainer::getLogger(), BSPContainer::getInterlocManager(), BSPContainer::getBSP(),
        MessageHandlerContainer::getInterlocMsgQueue(), MessageHandlerContainer::getHostMsgQueue(),
        MessageHandlerContainer::getRemoteMsgQueue());

    return s_messageHandler;
}

ThreadSafeQueue<uint16_t>& InterlocContainer::getInterlocUpdateOutputQueue() {
    static Mutex s_mutex(10);
    static CircularQueueStack<uint16_t, gc_interlocPosUpdateMaxSize> s_interlocMsgQueue;
    static ThreadSafeQueue<uint16_t> s_interlocPosUpdateThreadQueue(s_interlocMsgQueue, s_mutex);

    return s_interlocPosUpdateThreadQueue;
}

NotificationQueue<InterlocUpdate>& InterlocContainer::getInterlocUpdateInputQueue() {
    static Mutex s_mutex(10);
    static CircularQueueStack<InterlocUpdate, gc_interlocPosUpdateMaxSize> s_interlocUpdateQueue;
    static ThreadSafeQueue<InterlocUpdate> s_interlocUpdateThreadQueue(s_interlocUpdateQueue,
                                                                       s_mutex);
    static ConditionVariable s_interlocUpdateConditionVariable;
    static NotificationQueue<InterlocUpdate> s_interlocUpdateNotificationQueue(
        s_interlocUpdateThreadQueue, s_interlocUpdateConditionVariable);

    return s_interlocUpdateNotificationQueue;
}