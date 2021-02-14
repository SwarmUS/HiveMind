#include <AbstractTask.h>
#include <Task.h>
#include <bittybuzz/BittyBuzzFactory.h>
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
                      BSPContainer::getBSP(),
                      m_logger,
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

class UartMessageDispatcher : public AbstractTask<20 * configMINIMAL_STACK_SIZE> {

  public:
    UartMessageDispatcher(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    ~UartMessageDispatcher() override = default;

    void task() override {
        auto& uart = BSPContainer::getHostUart();

        while (true) {
            if (uart.isConnected()) {
                HiveMindHostDeserializer deserializer(uart);
                MessageDispatcher messageDispatcher(MessageHandlerContainer::getBuzzMsgQueue(),
                                                    MessageHandlerContainer::getHostMsgQueue(),
                                                    MessageHandlerContainer::getRemoteMsgQueue(),
                                                    deserializer, BSPContainer::getBSP().getUUId(),
                                                    m_logger);

                while (true) {
                    if (!messageDispatcher.deserializeAndDispatch()) {
                        m_logger.log(LogLevel::Warn, "Fail to deserialize/dispatch uart");
                    }
                }
            }
            Task::delay(500);
        }
    }

  private:
    ILogger& m_logger;
};

class TCPMessageDispatcher : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {

  public:
    TCPMessageDispatcher(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    ~TCPMessageDispatcher() override = default;

  private:
    ILogger& m_logger;

    void task() override {
        while (true) {
            auto socket = SocketContainer::getHostClientSocket();
            if (socket) {
                HiveMindHostDeserializer deserializer(socket.value());
                MessageDispatcher messageDispatcher(MessageHandlerContainer::getBuzzMsgQueue(),
                                                    MessageHandlerContainer::getHostMsgQueue(),
                                                    MessageHandlerContainer::getRemoteMsgQueue(),
                                                    deserializer, BSPContainer::getBSP().getUUId(),
                                                    m_logger);

                while (true) {
                    if (!messageDispatcher.deserializeAndDispatch()) {
                        m_logger.log(LogLevel::Warn, "Fail to deserialize/dispatch TCP");
                    }
                }
            }
            Task::delay(500);
        }
    }
};

class UartMessageSender : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    UartMessageSender(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    ~UartMessageSender() override = default;

  private:
    ILogger& m_logger;

    void task() override {
        auto& uart = BSPContainer::getHostUart();

        while (true) {
            if (uart.isConnected()) {

                HiveMindHostSerializer serializer(uart);

                // TODO: For now the uart is considered remote
                MessageSender messageSender(MessageHandlerContainer::getRemoteMsgQueue(),
                                            serializer, m_logger);

                while (true) {
                    if (!messageSender.processAndSerialize()) {
                        m_logger.log(LogLevel::Warn, "Fail to process/serialize to uart");
                    }
                }
            }
            Task::delay(500);
        }
    }
};

class TCPMessageSender : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    TCPMessageSender(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    ~TCPMessageSender() override = default;

  private:
    ILogger& m_logger;

    void task() override {
        while (true) {
            auto socket = SocketContainer::getHostClientSocket();
            if (socket) {
                HiveMindHostSerializer serializer(socket.value());
                MessageSender messageSender(MessageHandlerContainer::getHostMsgQueue(), serializer,
                                            m_logger);

                while (true) {
                    if (!messageSender.processAndSerialize()) {
                        m_logger.log(LogLevel::Warn, "Fail to process/serialize to tcp");
                    }
                }
            }
            Task::delay(500);
        }
    }
};

int main(int argc, char** argv) {
    CmdLineArgs cmdLineArgs = {argc, argv};

    IBSP& bsp = BSPContainer::getBSP();
    bsp.initChip((void*)&cmdLineArgs);

    static BittyBuzzTask s_bittybuzzTask("bittybuzz", tskIDLE_PRIORITY + 1);
    static UartMessageDispatcher s_uartDispatchTask("uart_dispatch", tskIDLE_PRIORITY + 1);
    static TCPMessageDispatcher s_tcpDispatchTask("tcp_dispatch", tskIDLE_PRIORITY + 1);
    static UartMessageSender s_uartMessageSender("uart_send", tskIDLE_PRIORITY + 1);
    static TCPMessageSender s_tcpMessageSender("uart_send", tskIDLE_PRIORITY + 1);

    s_bittybuzzTask.start();
    s_uartDispatchTask.start();
    s_tcpDispatchTask.start();
    s_uartMessageSender.start();
    s_tcpMessageSender.start();

    Task::startScheduler();

    return 0;
}
