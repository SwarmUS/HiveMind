#include "bsp/bsp.h"

#include <FreeRTOS.h>

//#include <stm32f4xx_hal.h>
//#include <stm32f4xx_hal_uart.h>
#include <task.h>
#include <timers.h>

#include <hivemind_hal.h>

// STM32F4-Discovery green led - PD12
#define LED_PORT GPIOB
#define LED_PIN GPIO_PIN_7
#define LED_PORT_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE

void blinky(void* param) {
    const int toggle_delay = 500;
    while (true) {
        vTaskDelay(toggle_delay);
        HAL_UART_Transmit(huart_print, (uint8_t*)"I love boobies\n", sizeof("I love boobies\n"),
                          HAL_MAX_DELAY);
    }
}

void init_chip() {
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART3_UART_Init();

}
