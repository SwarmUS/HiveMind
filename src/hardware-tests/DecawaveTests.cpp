#include "../bsp/src/stm32/src/interloc/include/interloc/DecawaveArray.h"
#include <AbstractTask.h>
#include <Task.h>
#include <bsp/BSPContainer.h>
#include <bsp/IBSP.h>
#include <hal/user_interface.h>
#include <logger/LoggerContainer.h>

static DecawaveArray g_decaArray;

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
        g_decaArray.initializeAll();
        if (!g_decaArray.canDoTWR()) {
            UI_setRGB(true, false, false); // Set RGB red
            return;
        }

        // If only one BB plugged in, will return the channel that is used
        Decawave& deca = g_decaArray.getMasterAntenna()->get();
        UI_setRGB(false, false, true); // Set RGB blue

        while (true) {
            int size = snprintf(m_buffer, sizeof(m_buffer), "Hello World #%d\n", m_numTransmits);
            deca.transmit((uint8_t*)m_buffer, size);
            UI_setHexOutput(static_cast<uint8_t>(m_numTransmits % 0x0F));
            m_numTransmits++;
            Task::delay(500);
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
        g_decaArray.initializeAll();
        if (!g_decaArray.canDoTWR()) {
            UI_setRGB(true, false, false); // Set RGB red
            return;
        }

        // If only one BB plugged in, will return the channel that is used
        Decawave& deca = g_decaArray.getMasterAntenna()->get();
        UI_setRGB(false, true, false); // Set RGB green

        uint8_t receivedMsgs = 0;

        while (true) {
            deca.receive(m_rxFrame, 2000);

            if (m_rxFrame.m_status == UWBRxStatus::FINISHED) {
                m_logger.log(LogLevel::Info, "Received UWB message: %s",
                             m_rxFrame.m_rxBuffer.data());
                receivedMsgs++;

                UI_setHexOutput(receivedMsgs % 0x0F);
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
        g_decaArray.initializeAll();
        if (!g_decaArray.canDoTWR()) {
            UI_setRGB(true, false, false); // Set RGB red
            return;
        }

        // If only one BB plugged in, will return the channel that is used
        Decawave& deca = g_decaArray.getMasterAntenna()->get();
        UI_setRGB(true, true, false);

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
            UI_setHexOutput(static_cast<uint8_t>(m_numWrites % 0x0F));
            m_numWrites++;
            Task::delay(1000);
        }
    }
};

class LedTest : public AbstractTask<10 * configMINIMAL_STACK_SIZE> {
  public:
    LedTest(const char* taskName, UBaseType_t priority) :
        AbstractTask(taskName, priority), m_logger(LoggerContainer::getLogger()) {}

    ~LedTest() override = default;

  private:
    ILogger& m_logger;
    std::array<DW_LED, 2> m_ledsToTest = {DW_LED::LED_2, DW_LED::LED_3};

    void task() override {
        g_decaArray.initializeAll();
        if (!g_decaArray.canDoTWR()) {
            UI_setRGB(true, false, false); // Set RGB red
            return;
        }

        // If only one BB plugged in, will return the channel that is used
        Decawave& deca = g_decaArray.getMasterAntenna()->get();
        UI_setRGB(false, true, true);

        while (true) {
            for (auto led : m_ledsToTest) {
                deca.setLed(led, true);
                Task::delay(500);
            }

            for (auto led : m_ledsToTest) {
                deca.setLed(led, false);
                Task::delay(500);
            }
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

#ifdef DECA_TEST_LED
    static LedTest s_ledTask("led_task", 10);
    s_ledTask.start();
#endif

    Task::startScheduler();

    return 0;
}