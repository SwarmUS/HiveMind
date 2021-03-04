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
                HiveMindApiRequestHandler hivemindApiReqHandler =
                    MessageHandlerContainer::createHiveMindApiRequestHandler();
                MessageDispatcher messageDispatcher =
                    MessageHandlerContainer::createMessageDispatcher(deserializer,
                                                                     hivemindApiReqHandler);
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
                HiveMindApiRequestHandler hivemindApiReqHandler =
                    MessageHandlerContainer::createHiveMindApiRequestHandler();
                MessageDispatcher messageDispatcher =
                    MessageHandlerContainer::createMessageDispatcher(deserializer,
                                                                     hivemindApiReqHandler);

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
                                            serializer, BSPContainer::getBSP(), m_logger);
                // TODO: The uart needs to receive byte to know it's connected, so both sides would
                // wait for message. Find a fix for that
                messageSender.greet();
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
                                            BSPContainer::getBSP(), m_logger);

                messageSender.greet();
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

class SPIMessageSender : public AbstractTask<5 * configMINIMAL_STACK_SIZE> {
  public:
    SPIMessageSender(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    ~SPIMessageSender() override = default;

  private:
    ILogger& m_logger;

    void task() override {
        auto& spi = BSPContainer::getSpiEsp();
        while (true) {
            if (!spi.isBusy()) {
                const char message[] = "Sending message to ESP from task";
                spi.send((uint8_t*)message, sizeof(message));
            }
            Task::delay(100);
        }
    }
};

int main(int argc, char** argv) {
    CmdLineArgs cmdLineArgs = {argc, argv};

    IBSP& bsp = BSPContainer::getBSP();
    bsp.initChip((void*)&cmdLineArgs);

    static BittyBuzzTask s_bittybuzzTask("bittybuzz", tskIDLE_PRIORITY + 1);
    static UartMessageDispatcher s_uartDispatchTask("uart_dispatch", tskIDLE_PRIORITY + 1);
//    static TCPMessageDispatcher s_tcpDispatchTask("tcp_dispatch", tskIDLE_PRIORITY + 1);
    static UartMessageSender s_uartMessageSender("uart_send", tskIDLE_PRIORITY + 1);
//    static TCPMessageSender s_tcpMessageSender("uart_send", tskIDLE_PRIORITY + 1);
    static SPIMessageSender s_spiMessageSender("spi_send", tskIDLE_PRIORITY + 1);

    s_bittybuzzTask.start();
    s_uartDispatchTask.start();
//    s_tcpDispatchTask.start();
    s_uartMessageSender.start();
//    s_tcpMessageSender.start();
    // s_spiMessageSender.start();

    Task::startScheduler();

    return 0;
}
