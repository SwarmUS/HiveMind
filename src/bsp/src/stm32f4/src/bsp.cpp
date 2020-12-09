#include "bsp/bsp.h"

#include <FreeRTOS.h>

//#include <stm32f4xx_hal.h>
//#include <stm32f4xx_hal_uart.h>
#include <cstdint>
#include <task.h>
#include <timers.h>

#include <eth.h>
#include <gpio.h>
#include <main.h>
#include <stm32f4xx_it.h>
#include <usart.h>
#include <usb_otg.h>

// STM32F4-Discovery green led - PD12
#define LED_PORT GPIOB
#define LED_PIN GPIO_PIN_7
#define LED_PORT_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE

void blinky(void* param) {
    const int toggle_delay = 500;
    while (true) {
        vTaskDelay(toggle_delay);
        HAL_UART_Transmit(&huart3, (uint8_t*)"I love boobies\n", sizeof("I love boobies\n"),
                          HAL_MAX_DELAY);
    }
}

void init_chip() {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */
    HardFault_Handler();
    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ETH_Init();
    MX_USART3_UART_Init();
    MX_USB_OTG_FS_PCD_Init();
    /* USER CODE BEGIN 2 */


    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}
