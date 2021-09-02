#include "deca_platform.h"
#include "deca_port.h"
#include <FreeRTOS.h>
#include <task.h>

beeboardChannelConfig_t g_beeboardChannelsConfig[DWT_NUM_DW_DEV];

void deca_setSync(bool state) {
    HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void deca_pulseSyncSignal() {
    deca_setSync(true);
    vTaskDelay(1);
    deca_setSync(false);
}

bool beeboard_isChannelPopulated(decaDevice_t channel) {
    if (channel >= DWT_NUM_DW_DEV) {
        return false;
    }

    return (HAL_GPIO_ReadPin(g_beeboardChannelsConfig[channel].m_channelDetectPort,
                             g_beeboardChannelsConfig[channel].m_channelDetectPin) == GPIO_PIN_SET);
}

void beeboard_enableChannel(decaDevice_t channel) {
    HAL_GPIO_WritePin(g_beeboardChannelsConfig[channel].m_channelEnablePort,
                      g_beeboardChannelsConfig[channel].m_channelEnablePin, GPIO_PIN_SET);
}

void beeboard_disableChannel(decaDevice_t channel) {
    HAL_GPIO_WritePin(g_beeboardChannelsConfig[channel].m_channelEnablePort,
                      g_beeboardChannelsConfig[channel].m_channelEnablePin, GPIO_PIN_RESET);
}

void beeboard_enableClock(decaDevice_t channel) {
    HAL_GPIO_WritePin(g_beeboardChannelsConfig[channel].m_nClockEnablePort,
                      g_beeboardChannelsConfig[channel].m_nClockEnablePin, GPIO_PIN_RESET);
}

void beeboard_disableClock(decaDevice_t channel) {
    HAL_GPIO_WritePin(g_beeboardChannelsConfig[channel].m_nClockEnablePort,
                      g_beeboardChannelsConfig[channel].m_nClockEnablePin, GPIO_PIN_SET);
}

void deca_init() {
    if (!channels_powerEnabled()) {
        return;
    }

    for (decaDevice_t channel = 0; channel < DWT_NUM_DW_DEV; channel++) {
        if (beeboard_isChannelPopulated(channel)) {
            g_decawaveConfigs[channel].isPresent = true;
            beeboard_enableChannel(channel);
            HAL_Delay(500); // Leave time for BB to detect USB-C orientation

            beeboard_enableClock(channel);

            deca_hardwareReset(channel);
            deca_setSlowRate(channel);
        }
    }

    HAL_Delay(1000);
}

void deca_hardwareReset(decaDevice_t selectedDevice) {
    decawaveDeviceConfig_t* decaConfig = deca_getDeviceConfig(selectedDevice);

    HAL_GPIO_WritePin(decaConfig->resetPort, decaConfig->resetPin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(decaConfig->resetPort, decaConfig->resetPin, GPIO_PIN_SET);
}

bool channels_powerEnabled() {
    return HAL_GPIO_ReadPin(CH_CL_PWR_EN_GPIO_Port, CH_CL_PWR_EN_Pin) == GPIO_PIN_SET;
}

decawaveDeviceConfig_t g_decawaveConfigs[DWT_NUM_DW_DEV] = {{.spiHandle = &hspi1, // A0
                                                             .nssPort = SPI_nCS_0A_GPIO_Port,
                                                             .nssPin = SPI_nCS_0A_Pin,
                                                             .irqPort = CHAN_IRQ_0A_GPIO_Port,
                                                             .irqPin = CHAN_IRQ_0A_Pin,
                                                             .resetPort = CHAN_RESET_0A_GPIO_Port,
                                                             .resetPin = CHAN_RESET_0A_Pin,
                                                             .isrCallback = NULL,
                                                             .isrContext = NULL,
                                                             .isPresent = false},
                                                            {.spiHandle = &hspi1, // A1
                                                             .nssPort = SPI_nCS_1A_GPIO_Port,
                                                             .nssPin = SPI_nCS_1A_Pin,
                                                             .irqPort = CHAN_IRQ_1A_GPIO_Port,
                                                             .irqPin = CHAN_IRQ_1A_Pin,
                                                             .resetPort = CHAN_RESET_1A_GPIO_Port,
                                                             .resetPin = CHAN_RESET_1A_Pin,
                                                             .isrCallback = NULL,
                                                             .isrContext = NULL,
                                                             .isPresent = false},
                                                            {.spiHandle = &hspi3, // B0
                                                             .nssPort = SPI_nCS_0B_GPIO_Port,
                                                             .nssPin = SPI_nCS_0B_Pin,
                                                             .irqPort = CHAN_IRQ_0B_GPIO_Port,
                                                             .irqPin = CHAN_IRQ_0B_Pin,
                                                             .resetPort = CHAN_RESET_0B_GPIO_Port,
                                                             .resetPin = CHAN_RESET_0B_Pin,
                                                             .isrCallback = NULL,
                                                             .isrContext = NULL,
                                                             .isPresent = false},
                                                            {.spiHandle = &hspi3, // B1
                                                             .nssPort = SPI_nCS_1B_GPIO_Port,
                                                             .nssPin = SPI_nCS_1B_Pin,
                                                             .irqPort = CHAN_IRQ_1B_GPIO_Port,
                                                             .irqPin = CHAN_IRQ_1B_Pin,
                                                             .resetPort = CHAN_RESET_1B_GPIO_Port,
                                                             .resetPin = CHAN_RESET_1B_Pin,
                                                             .isrCallback = NULL,
                                                             .isrContext = NULL,
                                                             .isPresent = false},
                                                            {.spiHandle = &hspi2, // C0
                                                             .nssPort = SPI_nCS_0C_GPIO_Port,
                                                             .nssPin = SPI_nCS_0C_Pin,
                                                             .irqPort = CHAN_IRQ_0C_GPIO_Port,
                                                             .irqPin = CHAN_IRQ_0C_Pin,
                                                             .resetPort = CHAN_RESET_0C_GPIO_Port,
                                                             .resetPin = CHAN_RESET_0C_Pin,
                                                             .isrCallback = NULL,
                                                             .isrContext = NULL,
                                                             .isPresent = false},
                                                            {.spiHandle = &hspi2, // C1
                                                             .nssPort = SPI_nCS_1C_GPIO_Port,
                                                             .nssPin = SPI_nCS_1C_Pin,
                                                             .irqPort = CHAN_IRQ_1C_GPIO_Port,
                                                             .irqPin = CHAN_IRQ_1C_Pin,
                                                             .resetPort = CHAN_RESET_1C_GPIO_Port,
                                                             .resetPin = CHAN_RESET_1C_Pin,
                                                             .isrCallback = NULL,
                                                             .isrContext = NULL,
                                                             .isPresent = false}};

beeboardChannelConfig_t g_beeboardChannelsConfig[DWT_NUM_DW_DEV] = {
    {.m_channelDetectPort = CHAN_DET_0A_GPIO_Port, // 0A
     .m_channelDetectPin = CHAN_DET_0A_Pin,
     .m_channelEnablePort = CHAN_EN_0A_GPIO_Port,
     .m_channelEnablePin = CHAN_EN_0A_Pin,
     .m_nClockEnablePort = CLK_OE_0__GPIO_Port,
     .m_nClockEnablePin = CLK_OE_0__Pin},
    {.m_channelDetectPort = CHAN_DET_1A_GPIO_Port, // 1A
     .m_channelDetectPin = CHAN_DET_1A_Pin,
     .m_channelEnablePort = CHAN_EN_1A_GPIO_Port,
     .m_channelEnablePin = CHAN_EN_1A_Pin,
     .m_nClockEnablePort = CLK_OE_1__GPIO_Port,
     .m_nClockEnablePin = CLK_OE_1__Pin},
    {.m_channelDetectPort = CHAN_DET_0B_GPIO_Port, // 0B
     .m_channelDetectPin = CHAN_DET_0B_Pin,
     .m_channelEnablePort = CHAN_EN_0B_GPIO_Port,
     .m_channelEnablePin = CHAN_EN_0B_Pin,
     .m_nClockEnablePort = CLK_OE_0__GPIO_Port,
     .m_nClockEnablePin = CLK_OE_0__Pin},
    {.m_channelDetectPort = CHAN_DET_1B_GPIO_Port, // 1B
     .m_channelDetectPin = CHAN_DET_1B_Pin,
     .m_channelEnablePort = CHAN_EN_1B_GPIO_Port,
     .m_channelEnablePin = CHAN_EN_1B_Pin,
     .m_nClockEnablePort = CLK_OE_1__GPIO_Port,
     .m_nClockEnablePin = CLK_OE_1__Pin},
    {.m_channelDetectPort = CHAN_DET_0C_GPIO_Port, // 0C
     .m_channelDetectPin = CHAN_DET_0C_Pin,
     .m_channelEnablePort = CHAN_EN_0C_GPIO_Port,
     .m_channelEnablePin = CHAN_EN_0C_Pin,
     .m_nClockEnablePort = CLK_OE_0__GPIO_Port,
     .m_nClockEnablePin = CLK_OE_0__Pin},
    {.m_channelDetectPort = CHAN_DET_1C_GPIO_Port, // 1C
     .m_channelDetectPin = CHAN_DET_1C_Pin,
     .m_channelEnablePort = CHAN_EN_1C_GPIO_Port,
     .m_channelEnablePin = CHAN_EN_1C_Pin,
     .m_nClockEnablePort = CLK_OE_1__GPIO_Port,
     .m_nClockEnablePin = CLK_OE_1__Pin}

};
