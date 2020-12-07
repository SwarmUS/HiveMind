#include "bsp/bsp.h"

#include <FreeRTOS.h>

#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_uart.h>
#include <task.h>
#include <timers.h>

#include <usart.h>
#include <gpio.h>
#include <eth.h>
#include <usb_otg.h>
#include <main.h>
// STM32F4-Discovery green led - PD12
#define LED_PORT GPIOB
#define LED_PIN GPIO_PIN_7
#define LED_PORT_CLK_ENABLE __HAL_RCC_GPIOB_CLK_ENABLE

void blinky(void* param) {
    const int toggle_delay = 500;
    while (true) {
        vTaskDelay(toggle_delay);
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    }
}


void init_chip() {
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();
    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    //MX_ETH_Init();
    MX_USART3_UART_Init();
    //MX_USB_OTG_FS_PCD_Init();


    while(1){
        int test = 0;
        for (int i =0; i<20; i++){
            test += HAL_GetTick();
        }
        HAL_Delay(test);
        HAL_UART_Transmit(&huart3, (uint8_t*) "I love boobies\n", sizeof("I love boobies\n"), HAL_MAX_DELAY);
    }
    
}
