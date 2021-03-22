#include <AbstractTask.h>
#include <Task.h>
#include <bittybuzz/BittyBuzzContainer.h>
#include <bittybuzz/BittyBuzzFactory.h>
#include <bittybuzz/BittyBuzzMessageHandler.h>
#include <bittybuzz/BittyBuzzVm.h>
#include <bsp/BSPContainer.h>
#include <bsp/IBSP.h>
#include <bsp/SocketContainer.h>
#include <cstdlib>
#include <hivemind-host/HiveMindHostDeserializer.h>
#include <hivemind-host/HiveMindHostSerializer.h>
#include <logger/Logger.h>
#include <logger/LoggerContainer.h>
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

class HostUsbConnexionGreet : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  private:
    void task() override {
        auto& usb = BSPContainer::getUSB();
        while (true) {
            if (usb.isConnected()) {
                m_hostStream = &usb;
                m_logger.log(LogLevel::Info, "Host connected via USB");
            }
        }
    }
};

class HostMessageDispatcher : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    HostMessageDispatcher(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    ~HostMessageDispatcher() override = default;

    IProtobufStream* m_hostStream = NULL;

  private:
    ILogger& m_logger;

    void task() override {

        auto& usb = BSPContainer::getUSB();
        while (true) {

            auto socket = SocketContainer::getHostClientSocket();

            if (socket) {
                m_hostStream = &socket.value();
                m_logger.log(LogLevel::Info, "Host connected via TCP");
            } else {
                if (usb.isConnected()) {
                    m_hostStream = &usb;
                    m_logger.log(LogLevel::Info, "Host connected via USB");
                }
            }

            if (m_hostStream != NULL) {
                HiveMindHostDeserializer deserializer(*m_hostStream);
                HiveMindApiRequestHandler hivemindApiReqHandler =
                    MessageHandlerContainer::createHiveMindApiRequestHandler();
                MessageDispatcher messageDispatcher =
                    MessageHandlerContainer::createMessageDispatcher(deserializer,
                                                                     hivemindApiReqHandler);
                while (m_hostStream != NULL) {
                    if (!messageDispatcher.deserializeAndDispatch()) {
                        m_logger.log(LogLevel::Warn, "Fail to deserialize/dispatch to host");
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

            if (m_messageDispatcher.m_hostStream != NULL) {
                HiveMindHostSerializer serializer(*m_messageDispatcher.m_hostStream);
                MessageSender messageSender(MessageHandlerContainer::getHostMsgQueue(), serializer,
                                            BSPContainer::getBSP(), m_logger);
                messageSender.greet();
                while (m_messageDispatcher.m_hostStream != NULL) {
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

        auto& spi = BSPContainer::getSpiEsp();
        while (true) {
            if (spi.isConnected()) {

                HiveMindHostSerializer serializer(spi);
                MessageSender messageSender(MessageHandlerContainer::getRemoteMsgQueue(),
                                            serializer, BSPContainer::getBSP(), m_logger);

                while (true) {
                    if (!messageSender.processAndSerialize()) {
                        m_logger.log(LogLevel::Warn, "Fail to process/serialize to remote");
                    }
                }
            }
            Task::delay(500);
        }
    }
};

class RemoteMessageDispatcher : public AbstractTask<20 * configMINIMAL_STACK_SIZE> {

  public:
    RemoteMessageDispatcher(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    ~RemoteMessageDispatcher() override = default;

    void task() override {
        auto& spi = BSPContainer::getSpiEsp();
        while (!spi.isConnected()) {
            Task::delay(500);
        }
        HiveMindHostDeserializer deserializer(spi);
        auto hivemindApiReqHandler = MessageHandlerContainer::createHiveMindApiRequestHandler();
        auto messageDispatcher =
            MessageHandlerContainer::createMessageDispatcher(deserializer, hivemindApiReqHandler);
        while (true) {
            if (!messageDispatcher.deserializeAndDispatch()) {
                m_logger.log(LogLevel::Warn, "Fail to deserialize/dispatch spi");
            }
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
    static RemoteMessageDispatcher s_spiDispatchTask("spi_dispatch", tskIDLE_PRIORITY + 1);
    static RemoteMessageSender s_spiMessageSender("spi_send", tskIDLE_PRIORITY + 1);
    static InterlocTask s_interlocTask("interloc", tskIDLE_PRIORITY + 5);

    s_bittybuzzTask.start();
    s_hostDispatchTask.start();
    s_hostMessageSender.start();
    s_spiMessageSender.start();
    s_interlocTask.start();

    Task::startScheduler();

    return 0;
}
