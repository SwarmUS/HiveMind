#include "bsp.h"

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <stm32f4xx_hal.h>
#include <task.h>
#include <timers.h>

// STM32F4-Discovery green led - PD12
#define LED_PORT GPIOD
#define LED_PIN GPIO_PIN_12
#define LED_PORT_CLK_ENABLE __HAL_RCC_GPIOD_CLK_ENABLE

void blinky() {
  for (;;) {
    vTaskDelay(500);
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
  }
}

void initGPIO() {
  GPIO_InitTypeDef GPIO_Config;

  GPIO_Config.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_Config.Pull = GPIO_NOPULL;
  GPIO_Config.Speed = GPIO_SPEED_FREQ_HIGH;

  GPIO_Config.Pin = LED_PIN;

  LED_PORT_CLK_ENABLE();
  HAL_GPIO_Init(LED_PORT, &GPIO_Config);
}

void init_chip() {
  SystemInit();
  initGPIO();
}
