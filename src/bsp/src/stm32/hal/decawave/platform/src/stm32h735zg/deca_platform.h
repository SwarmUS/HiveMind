#ifndef HIVE_MIND_DECA_PLATFORM_H
#define HIVE_MIND_DECA_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "hivemind_hal.h"
#include <stdbool.h>

// TODO: Set to correct prescalers (maximum ratings: slow=3MHz, fast=20MHz)
// SPI Base clock = 64 MHz
#define DECA_SPI_SLOW_RATE SPI_BAUDRATEPRESCALER_32
#define DECA_SPI_FAST_RATE SPI_BAUDRATEPRESCALER_8

#define DECA_IRQ_IDLE_STATE GPIO_PIN_SET

/**
 * @brief Enum to specify which decawave we are addressing
 */
typedef enum { DW_A0 = 0, DW_A1, DW_B0, DW_B1, DW_C0, DW_C1 } decaDevice_t;

/**
 * @brief Struct containing pins needed to handle a beeboard channel
 */
typedef struct {
    GPIO_TypeDef* m_channelDetectPort;
    uint16_t m_channelDetectPin;
    GPIO_TypeDef* m_channelEnablePort;
    uint16_t m_channelEnablePin;
    GPIO_TypeDef* m_nClockEnablePort;
    uint16_t m_nClockEnablePin;
} beeboardChannelConfig_t;

/**
 * @brief Sets the sync pin
 * @param state State to set the pin
 */
void deca_setSync(bool state);

/**
 * @brief Tells if a beeboard is currently connected to a channel
 * @param channel Channel to check
 * @return True if connected, false otherwise
 */
bool beeboard_isChannelPopulated(decaDevice_t channel);

/**
 * @brief Enables the channel hardware (except for the clock)
 * @param channel Channel to enable
 */
void beeboard_enableChannel(decaDevice_t channel);

/**
 * @brief Disables the channel hardware (except for the clock)
 * @param channel Channel to disable
 */
void beeboard_disableChannel(decaDevice_t channel);

/**
 * @brief Enables the clock for a given channel tree
 * @param channel Channel to enable the clock for
 */
void beeboard_enableClock(decaDevice_t channel);

/**
 * @brief Disables the clock for a given channel tree
 * @param channel Channel to disable the clock for
 */
void beeboard_disableClock(decaDevice_t channel);

/**
 * @brief Tells if channel hardware has power running to it or not
 * @return True if powered
 */
bool channels_powerEnabled();

#ifdef __cplusplus
}
#endif

#endif // HIVE_MIND_DECA_PLATFORM_H
