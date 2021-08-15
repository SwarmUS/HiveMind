#include "TestChannels.h"
#include <deca_platform.h>
#include <deca_port.h>

void TestChannels::runTests() {
    for (int i = 0; i < 6; i++) {
        decaDevice_t channel = (decaDevice_t)i;

        test01_Detect(channel);
        test02_Enable(channel);
        test03_Reset(channel);

        // Hard to test with breakout. Uncomment only if you know what you are doing
        // test04_IRQ(channel);
        // test05_SPI(channel);
        test06_CS(channel);

        beeboard_disableChannel(channel);
    }
}

void TestChannels::test01_Detect(decaDevice_t channel) {
    volatile bool isPopulated = beeboard_isChannelPopulated(channel);
    if (!isPopulated) {
        while (true) {
        }
    }
}

void TestChannels::test02_Enable(decaDevice_t channel) { beeboard_enableChannel(channel); }

void TestChannels::test03_Reset(decaDevice_t channel) {
    // Step through deca_hardwareReset() to control the state of the reset pin
    deca_hardwareReset(channel);
}

void TestChannels::test04_IRQ(decaDevice_t channel) {
    deca_setISRCallback(channel, isr, this);
    g_decawaveConfigs[channel].isPresent = true;

    // Pulse IRQ to continue
    while (!m_isrCalled) {
        HAL_Delay(100);
    }

    // Reset for next test
    m_isrCalled = false;
}

void TestChannels::test05_SPI(decaDevice_t channel) {
    uint8_t txBuffer[256];
    uint8_t rxBuffer[256];

    for (int i = 0; i < 256; i++) {
        txBuffer[i] = i;
    }

    HAL_SPI_TransmitReceive(g_decawaveConfigs[channel].spiHandle, txBuffer, rxBuffer, 256,
                            HAL_MAX_DELAY);

    // Check byte by byte if message was correctly loopedback
    for (int i = 0; i < 256; i++) {
        if (rxBuffer[i] != txBuffer[i]) {
            while (true) {
            }
        }
    }
}

void TestChannels::test06_CS(decaDevice_t channel) {
    HAL_GPIO_TogglePin(g_decawaveConfigs[channel].nssPort, g_decawaveConfigs[channel].nssPin);
    HAL_GPIO_TogglePin(g_decawaveConfigs[channel].nssPort, g_decawaveConfigs[channel].nssPin);
}

void TestChannels::isr(void* context) { static_cast<TestChannels*>(context)->m_isrCalled = true; }
