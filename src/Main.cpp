#include <AbstractTask.h>
#include <Task.h>
#include <bittybuzz/BittyBuzzContainer.h>
#include <bittybuzz/BittyBuzzFactory.h>
#include <bittybuzz/BittyBuzzMessageHandler.h>
#include <bittybuzz/BittyBuzzVm.h>
#include <bsp/BSPContainer.h>
#include <bsp/IBSP.h>
#include <cstdlib>
#include <hivemind-host/HiveMindHostDeserializer.h>
#include <hivemind-host/HiveMindHostSerializer.h>
#include <logger/Logger.h>
#include <logger/LoggerContainer.h>
#include <message-handler/GreetHandler.h>
#include <message-handler/GreetSender.h>
#include <message-handler/MessageDispatcher.h>
#include <message-handler/MessageHandlerContainer.h>
#include <message-handler/MessageSender.h>

class BittyBuzzTask : public AbstractTask<6 * configMINIMAL_STACK_SIZE> {
  public:
    BittyBuzzTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority),
        m_logger(LoggerContainer::getLogger()),
        m_bytecode(BittyBuzzFactory::createBittyBuzzBytecode(m_logger)),
        m_stringResolver(BittyBuzzFactory::createBittyBuzzStringResolver(m_logger)),
        m_bittybuzzVm(m_bytecode,
                      m_stringResolver,
                      BittyBuzzContainer::getBBZMessageHandler(),
                      BittyBuzzContainer::getBBZClosureRegister(),
                      BittyBuzzContainer::getBBZMessageService(),
                      BSPContainer::getBSP(),
                      m_logger,
                      BSPContainer::getUserInterface(),
                      BittyBuzzFactory::createBittyBuzzFunctionRegisters()) {}

    ~BittyBuzzTask() override = default;

  private:
    ILogger& m_logger;
    BittyBuzzBytecode m_bytecode;
    BittyBuzzStringResolver m_stringResolver;
    BittyBuzzVm m_bittybuzzVm;

    void task() override {
        while (true) {

            if (!m_bittybuzzVm.step()) {
                m_logger.log(LogLevel::Error, "BBZVM failed to step! state: %d err: %d",
                             m_bittybuzzVm.getSate(), m_bittybuzzVm.getError());
            }
            Task::delay(1000);
        }
    }
};

class HostMessageDispatcher : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    HostMessageDispatcher(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    ~HostMessageDispatcher() override = default;

  private:
    ILogger& m_logger;

    void task() override {
        while (true) {

            auto hostOpt = BSPContainer::getHostCommInterface();
            if (hostOpt) {
                ICommInterface& hostStream = hostOpt.value();
                HiveMindHostDeserializer deserializer(hostStream);
                HiveMindHostSerializer serializer(hostStream);
                HiveMindApiRequestHandler hivemindApiReqHandler =
                    MessageHandlerContainer::createHiveMindApiRequestHandler();

                // Establishing greet handlshake
                GreetHandler greetHandler(serializer, deserializer, BSPContainer::getBSP());
                if (greetHandler.greet()) {

                    GreetSender greetSender(MessageHandlerContainer::getHostMsgQueue(),
                                            BSPContainer::getBSP());
                    MessageDispatcher messageDispatcher =
                        MessageHandlerContainer::createMessageDispatcher(
                            deserializer, hivemindApiReqHandler, greetSender);

                    while (hostStream.isConnected()) {
                        if (!messageDispatcher.deserializeAndDispatch()) {
                            m_logger.log(LogLevel::Warn, "Fail to deserialize/dispatch to host");
                        }
                    }
                }
            }
            Task::delay(500);
        }
    }
};

class HostMessageSender : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    HostMessageSender(const char* taskName,
                      UBaseType_t priority,
                      HostMessageDispatcher& dispatcher) :
        AbstractTask(taskName, priority),
        m_messageDispatcher(dispatcher),
        m_logger(LoggerContainer::getLogger()) {}

    ~HostMessageSender() override = default;

    HostMessageDispatcher& m_messageDispatcher;

  private:
    ILogger& m_logger;

    void task() override {
        while (true) {

            auto hostOpt = BSPContainer::getHostCommInterface();
            if (hostOpt) {
                ICommInterface& hostStream = hostOpt.value();
                HiveMindHostSerializer serializer(hostStream);
                MessageSender messageSender(MessageHandlerContainer::getHostMsgQueue(), serializer,
                                            BSPContainer::getBSP(), m_logger);
                while (hostStream.isConnected()) {
                    if (!messageSender.processAndSerialize()) {
                        m_logger.log(LogLevel::Warn, "Fail to process/serialize to tcp");
                    }
                }
            }
            Task::delay(500);
        }
    }
};

class RemoteMessageSender : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    RemoteMessageSender(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    ~RemoteMessageSender() override = default;

  private:
    ILogger& m_logger;

    void task() override {
        while (true) {

            auto remoteOpt = BSPContainer::getRemoteCommInterface();
            if (remoteOpt) {
                ICommInterface& remoteStream = remoteOpt.value();
                HiveMindHostSerializer serializer(remoteStream);
                MessageSender messageSender(MessageHandlerContainer::getHostMsgQueue(), serializer,
                                            BSPContainer::getBSP(), m_logger);
                while (remoteStream.isConnected()) {
                    if (!messageSender.processAndSerialize()) {
                        m_logger.log(LogLevel::Warn, "Fail to process/serialize to tcp");
                    }
                }
            }
            Task::delay(500);
        }
    };
};

class RemoteMessageDispatcher : public AbstractTask<20 * configMINIMAL_STACK_SIZE> {
  public:
    RemoteMessageDispatcher(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    ~RemoteMessageDispatcher() override = default;

    void task() override {
        while (true) {

            auto remoteOpt = BSPContainer::getRemoteCommInterface();
            if (remoteOpt) {
                ICommInterface& remoteStream = remoteOpt.value();
                HiveMindHostDeserializer deserializer(remoteStream);
                HiveMindHostSerializer serializer(remoteStream);
                HiveMindApiRequestHandler hivemindApiReqHandler =
                    MessageHandlerContainer::createHiveMindApiRequestHandler();

                // Establishing greet handlshake
                GreetHandler greetHandler(serializer, deserializer, BSPContainer::getBSP());
                if (greetHandler.greet()) {

                    GreetSender greetSender(MessageHandlerContainer::getRemoteMsgQueue(),
                                            BSPContainer::getBSP());
                    MessageDispatcher messageDispatcher =
                        MessageHandlerContainer::createMessageDispatcher(
                            deserializer, hivemindApiReqHandler, greetSender);

                    while (remoteStream.isConnected()) {
                        if (!messageDispatcher.deserializeAndDispatch()) {
                            m_logger.log(LogLevel::Warn, "Fail to deserialize/dispatch to remote");
                        }
                    }
                }
            }
            Task::delay(500);
        }
    }

  private:
    ILogger& m_logger;
};

class InterlocTask : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    InterlocTask(const char* taskName, UBaseType_t priority) : AbstractTask(taskName, priority) {}

    ~InterlocTask() override = default;

  private:
    void task() override { BSPContainer::getInterlocManager().startInterloc(); }
};

int main(int argc, char** argv) {
    CmdLineArgs cmdLineArgs = {argc, argv};

    IBSP& bsp = BSPContainer::getBSP();
    bsp.initChip((void*)&cmdLineArgs);

    static BittyBuzzTask s_bittybuzzTask("bittybuzz", tskIDLE_PRIORITY + 1);
    static HostMessageDispatcher s_hostDispatchTask("tcp_dispatch", tskIDLE_PRIORITY + 1);
    static HostMessageSender s_hostMessageSender("host_send", tskIDLE_PRIORITY + 1,
                                                 s_hostDispatchTask);
    static RemoteMessageDispatcher s_remoteDispatchTask("remote_dispatch", tskIDLE_PRIORITY + 1);
    static RemoteMessageSender s_remoteMessageSender("remote_send", tskIDLE_PRIORITY + 1);
    static InterlocTask s_interlocTask("interloc", tskIDLE_PRIORITY + 5);

    s_bittybuzzTask.start();
    s_hostDispatchTask.start();
    s_hostMessageSender.start();
    s_remoteMessageSender.start();
    s_interlocTask.start();

    Task::startScheduler();

    return 0;
}
