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

class LoggerTask : public AbstractTask<configMINIMAL_STACK_SIZE> {
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

class BittyBuzzTask : public AbstractTask<8 * configMINIMAL_STACK_SIZE> {
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

class HostUartCommTask : public AbstractTask<configMINIMAL_STACK_SIZE> {
  public:
    HostUartCommTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority) {}

    ~HostUartCommTask() override = default;

  private:
    void task() override {
        IHostUart& hostUart = BSPContainer::getHostUart();
        while (true) {
            hostUart.send((const uint8_t*)"HELLO WORLD", sizeof("HELLO WORLD"));
            vTaskDelay(1000);
        }
    }
};

class HostTCPCommTask : public AbstractTask<configMINIMAL_STACK_SIZE> {
  public:
    HostTCPCommTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority) {}

    ~HostTCPCommTask() override = default;

  private:
    void task() override {

        // Wait for connection
        while (true) {
            if (std::optional<TCPClientWrapper> socket = SocketContainer::getHostClientSocket()) {
                while (true) {
                    auto ret =
                        socket.value().send((const uint8_t*)"HELLO WORLD", sizeof("HELLO WORLD"));
                    (void)ret;
                    vTaskDelay(1000);
                }
            }
        }
    }
};

int main(int argc, char** argv) {
    CmdLineArgs cmdLineArgs = {argc, argv};

    IBSP& bsp = BSPContainer::getBSP();
    bsp.initChip((void*)&cmdLineArgs);

    LoggerTask loggerTask("logger", tskIDLE_PRIORITY + 1);
    BittyBuzzTask bittybuzzTask("bittybuzz", tskIDLE_PRIORITY + 1);
    HostUartCommTask uartTask("uart", tskIDLE_PRIORITY + 1);
    HostTCPCommTask tcpTask("uart", tskIDLE_PRIORITY + 1);

    loggerTask.start();
    bittybuzzTask.start();
    uartTask.start();
    tcpTask.start();

    vTaskStartScheduler();

    return 0;
}
