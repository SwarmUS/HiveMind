#include "bsp/BSP.h"

#include <FreeRTOS.h>
#include <hivemind_hal.h>
#include <task.h>
#include <timers.h>

BSP::BSP() = default;
BSP::~BSP() = default;

void BSP::initChip() {
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART3_UART_Init();
}
