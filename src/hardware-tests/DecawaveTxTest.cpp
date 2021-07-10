#include "../bsp/src/stm32/src/interloc/include/interloc/Decawave.h"
#include <AbstractTask.h>
#include <Task.h>
#include <bsp/BSPContainer.h>
#include <bsp/IBSP.h>
#include <logger/LoggerContainer.h>

static decaDevice_t testedChannel = DW_A0;

class TxTask : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    TxTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority),
        m_logger(LoggerContainer::getLogger()),
        m_numTransmits(0) {}

    ~TxTask() override = default;

  private:
    ILogger& m_logger;
    char m_buffer[64];
    int m_numTransmits;

    void task() override {
        Decawave deca(testedChannel);

        deca.init();

        while (true) {
            int size = snprintf(m_buffer, sizeof(m_buffer), "Hello World #%d\n", m_numTransmits);
            deca.transmit((uint8_t*)m_buffer, size);
            m_numTransmits++;
            Task::delay(1000);
        }
    }
};

class RxTask : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    RxTask(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    ~RxTask() override = default;

  private:
    ILogger& m_logger;
    UWBRxFrame m_rxFrame;

    void task() override {
        Decawave deca(testedChannel);

        deca.init();

        while (true) {
            deca.receive(m_rxFrame, 2000);

            if (m_rxFrame.m_status == UWBRxStatus::FINISHED) {
                m_logger.log(LogLevel::Info, "Received UWB message: %s", m_rxFrame.m_rxBuffer);
            } else if (m_rxFrame.m_status == UWBRxStatus::ERROR) {
                m_logger.log(LogLevel::Error, "Error while receiving UWB message");
            }
        }
    }
};

class SpiTest : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    SpiTest(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()), m_numWrites(0) {}

    ~SpiTest() override = default;

  private:
    ILogger& m_logger;
    uint8_t m_writeBuffer[40];
    uint8_t m_readBackBuffer[40];
    int m_numWrites;

    void task() override {
        Decawave deca(testedChannel);
        deca.init();

        while (true) {
            for (uint8_t& val : m_writeBuffer) {
                val = rand();
            }
            dwt_writetodevice(0x21, 0, sizeof(m_writeBuffer), (uint8_t*)m_writeBuffer);

            dwt_readfromdevice(0x21, 0, sizeof(m_readBackBuffer), m_readBackBuffer);

            for (unsigned int i = 0; i < sizeof(m_writeBuffer); i++) {
                if (m_writeBuffer[i] != m_readBackBuffer[i]) {
                    m_logger.log(LogLevel::Error, "Byte #%d does not match on loop #%d", i,
                                 m_numWrites);
                    return;
                }
            }

            m_logger.log(LogLevel::Info, "Read back #%d successful", m_numWrites);
            m_numWrites++;
            Task::delay(1000);
        }
    }
};

int main(int argc, char** argv) {
    CmdLineArgs cmdLineArgs = {argc, argv};

    IBSP& bsp = BSPContainer::getBSP();
    bsp.initChip((void*)&cmdLineArgs);

#ifdef DECA_TEST_TX
    static TxTask s_txTask("tx_task", 10);
    s_txTask.start();
#endif

#ifdef DECA_TEST_RX
    static RxTask s_rxTask("rx_task", 10);
    s_rxTask.start();
#endif

#ifdef DECA_TEST_SPI
    static SpiTest s_spiTask("spi_task", 10);
    s_spiTask.start();
#endif

    Task::startScheduler();

    return 0;
}