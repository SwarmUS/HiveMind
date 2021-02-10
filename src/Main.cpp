#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <task.h>
#include <timers.h>

#include <bittybuzz/BittyBuzzFactory.h>
#include <bittybuzz/BittyBuzzVm.h>
#include <bsp/BSPContainer.h>
#include <bsp/IBSP.h>
#include <bsp/SocketContainer.h>
#include <cstdlib>
#include <freertos-utils/AbstractTask.h>
#include <logger/Logger.h>
#include <logger/LoggerContainer.h>

class LoggerTask : public AbstractTask<2 * configMINIMAL_STACK_SIZE> {
  public:
    LoggerTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    ~LoggerTask() override = default;

  private:
    ILogger& m_logger;
    void task() override {
        while (true) {
            m_logger.log(LogLevel::Info, "Hello logger");
            vTaskDelay(2000);
        }
    }
};

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
            vTaskDelay(1000);
        }
    }
};

class UARTReadTask : public AbstractTask<2 * configMINIMAL_STACK_SIZE> {
  public:
    UARTReadTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    ~UARTReadTask() override = default;

  private:
    ILogger& m_logger;
    void task() override {

        uint8_t buffer[5];
        buffer[sizeof(buffer) - 1] = 0;

        // Wait for connection
        while (true) {
            auto& uart = BSPContainer::getHostUart();
            if (uart.receive(buffer, sizeof(buffer) - 1)) {
                m_logger.log(LogLevel::Info, "UART RX: %s", buffer);
            } else {
                // Probably failed because no client connected
                vTaskDelay(2000);
            }
        }
    }
};

class HostUartCommTask : public AbstractTask<2 * configMINIMAL_STACK_SIZE> {
  public:
    HostUartCommTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority) {}

    ~HostUartCommTask() override = default;

  private:
    void task() override {
        IHostUart& hostUart = BSPContainer::getHostUart();
        uint8_t buffer[5];
        buffer[sizeof(buffer) - 1] = 0;

        while (true) {
            hostUart.send((const uint8_t*)"HELLO WORLD", sizeof("HELLO WORLD"));
            vTaskDelay(1000);
        }
    }
};

class TCPReadDemoTask : public AbstractTask<2 * configMINIMAL_STACK_SIZE> {
  public:
    TCPReadDemoTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    ~TCPReadDemoTask() override = default;

  private:
    ILogger& m_logger;
    void task() override {

        uint8_t buffer[5];
        buffer[sizeof(buffer) - 1] = 0;

        // Wait for connection
        while (true) {
            if (std::optional<TCPClientWrapper> socket = SocketContainer::getHostClientSocket()) {
                while (true) {
                    auto ret = socket.value().receive(buffer, sizeof(buffer) - 1);
                    (void)ret;
                    m_logger.log(LogLevel::Info, "TCP RX: %s", buffer);
                }
            }
            // Retry connection every 2s
            vTaskDelay(2000);
        }
    }
};

class HostTCPCommTask : public AbstractTask<2 * configMINIMAL_STACK_SIZE> {
  public:
    HostTCPCommTask(const char* taskName, UBaseType_t priority, TCPReadDemoTask& tcpReadTask) :
        AbstractTask(taskName, priority), m_tcpReadDemoTask(tcpReadTask) {}

    ~HostTCPCommTask() override = default;

  private:
    TCPReadDemoTask& m_tcpReadDemoTask;
    void task() override {

        // Wait for connection
        while (true) {
            if (std::optional<TCPClientWrapper> socket = SocketContainer::getHostClientSocket()) {
                m_tcpReadDemoTask.start();
                while (true) {
                    auto ret =
                        socket.value().send((const uint8_t*)"HELLO WORLD", sizeof("HELLO WORLD"));
                    (void)ret;
                    vTaskDelay(1000);
                }
            }
            // Retry connection every 2s
            vTaskDelay(2000);
        }
    }
};

int main(int argc, char** argv) {
    CmdLineArgs cmdLineArgs = {argc, argv};

    IBSP& bsp = BSPContainer::getBSP();
    bsp.initChip((void*)&cmdLineArgs);

    static LoggerTask s_loggerTask("logger", tskIDLE_PRIORITY + 1);
    static BittyBuzzTask s_bittybuzzTask("bittybuzz", tskIDLE_PRIORITY + 1);
    static UARTReadTask s_uartReadTask("uart_read", tskIDLE_PRIORITY + 1);
    static HostUartCommTask s_uartTask("uart", tskIDLE_PRIORITY + 1);
    static TCPReadDemoTask s_tcpReadTask("tcp_read", tskIDLE_PRIORITY + 1);
    static HostTCPCommTask s_tcpTask("tcp", tskIDLE_PRIORITY + 1, s_tcpReadTask);

    s_loggerTask.start();
    s_bittybuzzTask.start();
    s_uartReadTask.start();
    s_uartTask.start();
    s_tcpTask.start();

    vTaskStartScheduler();

    return 0;
}
