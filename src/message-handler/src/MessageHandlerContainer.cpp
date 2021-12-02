#include "MessageHandlerContainer.h"
#include "IGreetHandler.h"
#include <ConditionVariable.h>
#include <Mutex.h>
#include <bsp/BSPContainer.h>
#include <cpp-common/CircularQueueStack.h>
#include <interloc/InterlocContainer.h>
#include <logger/LoggerContainer.h>
#include <pheromones/HiveMindHostAccumulatorSerializer.h>
#include <pheromones/HiveMindHostSerializer.h>
#include <pheromones/HiveMindHostDeserializer.h>
#include "GreetSender.h"
#include "Task.h"

constexpr uint16_t gc_queueMaxSize = 16;

HiveConnectHiveMindApiMessageHandler MessageHandlerContainer::
    createHiveConnectHiveMindApiMessageHandler() {
    return HiveConnectHiveMindApiMessageHandler(getHostMsgQueue(), getRemoteMsgQueue(),
                                                LoggerContainer::getLogger());
}

HiveMindHostApiRequestHandler MessageHandlerContainer::createHiveMindHostApiRequestHandler() {
    return HiveMindHostApiRequestHandler(BSPContainer::getBSP(), getHostMsgQueue(),
                                         getRemoteMsgQueue(), InterlocContainer::getInterloc(),
                                         LoggerContainer::getLogger());
}

MessageDispatcher MessageHandlerContainer::createMessageDispatcher(
    IHiveMindHostDeserializer& deserializer,
    IHiveMindHostApiRequestHandler& hivemindApiReqHandler,
    IHiveConnectHiveMindApiMessageHandler& hiveconnectApiMessageHandler,
    IGreetSender& greetSender) {
    return MessageDispatcher(getBuzzMsgQueue(), getHostMsgQueue(), getRemoteMsgQueue(),
                             getInterlocMsgQueue(), deserializer, hivemindApiReqHandler,
                             hiveconnectApiMessageHandler, greetSender, BSPContainer::getBSP(),
                             LoggerContainer::getLogger());
}

MessageDispatcher& MessageHandlerContainer::createHostDispatcher() {
    while (!BSPContainer::getHostCommInterface().has_value()) {
        LoggerContainer::getLogger().log(LogLevel::Info, "Waiting on host interface");

    }
    static HiveMindHostDeserializer s_hostDeserializer(BSPContainer::getHostCommInterface().value());
    static GreetSender s_hostGreeter(getHostMsgQueue(), BSPContainer::getBSP());
    static HiveMindHostApiRequestHandler s_hmHandler(createHiveMindHostApiRequestHandler());
    static HiveConnectHiveMindApiMessageHandler s_hcHandler(createHiveConnectHiveMindApiMessageHandler());
    static MessageDispatcher s_hostDispatcher(
        createMessageDispatcher(s_hostDeserializer,s_hmHandler, s_hcHandler, s_hostGreeter));
    return s_hostDispatcher;

}
MessageDispatcher& MessageHandlerContainer::createRemoteDispatcher() {
    while (!BSPContainer::getRemoteCommInterface().has_value()) {
        LoggerContainer::getLogger().log(LogLevel::Info, "Waiting on remote interface");
    }
    static HiveMindHostDeserializer s_remoteDeserializer(BSPContainer::getRemoteCommInterface().value());
    static GreetSender s_remoteGreeter(getRemoteMsgQueue(), BSPContainer::getBSP());
    static HiveMindHostApiRequestHandler s_hmHandler(createHiveMindHostApiRequestHandler());
    static HiveConnectHiveMindApiMessageHandler s_hcHandler(createHiveConnectHiveMindApiMessageHandler());
    static MessageDispatcher s_remoteDispatcher(
        createMessageDispatcher(s_remoteDeserializer,s_hmHandler, s_hcHandler, s_remoteGreeter));
    return s_remoteDispatcher;
}
IHiveMindHostSerializer& MessageHandlerContainer::createHostSerializer() {
    while (!BSPContainer::getHostCommInterface().has_value()) {
        LoggerContainer::getLogger().log(LogLevel::Info, "Waiting on host interface");
    }
    static HiveMindHostAccumulatorSerializer s_hostSerializer(BSPContainer::getHostCommInterface().value());
    return s_hostSerializer;
}
IHiveMindHostSerializer& MessageHandlerContainer::createRemoteSerializer() {
    while (!BSPContainer::getRemoteCommInterface().has_value()) {
        LoggerContainer::getLogger().log(LogLevel::Info, "Waiting on remote interface");
    }
    static HiveMindHostAccumulatorSerializer s_remoteSerializer(BSPContainer::getRemoteCommInterface().value());
    return s_remoteSerializer;
}

ThreadSafeQueue<MessageDTO>& MessageHandlerContainer::getBuzzMsgQueue() {
    __attribute__((section(".cmbss"))) static Mutex s_mutex(10);
    __attribute__((section(".cmbss"))) static CircularQueueStack<MessageDTO, gc_queueMaxSize>
        s_buzzMsgQueue;
    static ThreadSafeQueue<MessageDTO> s_buzzMsgThreadQueue(s_buzzMsgQueue, s_mutex);

    return s_buzzMsgThreadQueue;
}

NotificationQueue<MessageDTO>& MessageHandlerContainer::getHostMsgQueue() {
    __attribute__((section(".cmbss"))) static Mutex s_mutex(10);
    __attribute__((section(".cmbss"))) static CircularQueueStack<MessageDTO, gc_queueMaxSize>
        s_hostMsgQueue;

    static ThreadSafeQueue<MessageDTO> s_hostMsgThreadQueue(s_hostMsgQueue, s_mutex);
    static ConditionVariable s_hostMsgConditionVariable;
    static NotificationQueue<MessageDTO> s_hostMsgNotificationQueue(s_hostMsgThreadQueue,
                                                                    s_hostMsgConditionVariable);

    return s_hostMsgNotificationQueue;
}

NotificationQueue<MessageDTO>& MessageHandlerContainer::getRemoteMsgQueue() {
    __attribute__((section(".cmbss"))) static Mutex s_mutex(10);
    __attribute__((section(".cmbss"))) static CircularQueueStack<MessageDTO, gc_queueMaxSize>
        s_remoteMsgQueue;

    static ThreadSafeQueue<MessageDTO> s_remoteMsgThreadQueue(s_remoteMsgQueue, s_mutex);
    static ConditionVariable s_remoteMsgConditionVariable;
    static NotificationQueue<MessageDTO> s_remoteMsgNotificationQueue(s_remoteMsgThreadQueue,
                                                                      s_remoteMsgConditionVariable);

    return s_remoteMsgNotificationQueue;
}

NotificationQueue<MessageDTO>& MessageHandlerContainer::getInterlocMsgQueue() {
    __attribute__((section(".cmbss"))) static Mutex s_mutex(10);
    __attribute__((section(".cmbss"))) static CircularQueueStack<MessageDTO, gc_queueMaxSize>
        s_interlocMsgQueue;

    static ThreadSafeQueue<MessageDTO> s_interlocMsgThreadQueue(s_interlocMsgQueue, s_mutex);
    static ConditionVariable s_interlocMsgConditionVariable;
    static NotificationQueue<MessageDTO> s_interlocMsgNotificationQueue(
        s_interlocMsgThreadQueue, s_interlocMsgConditionVariable);

    return s_interlocMsgNotificationQueue;
}
