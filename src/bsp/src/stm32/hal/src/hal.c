#include "hal/hal.h"
#include "hal/uart_print.h"
#include "hivemind_hal.h"

void Hal_init() {

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();

    MX_DMA_Init();

    MX_USART3_UART_Init();
    MX_USART2_UART_Init();

    /* Initialize UartPrint */
    UartPrint_init();
}