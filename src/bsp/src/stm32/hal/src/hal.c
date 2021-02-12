#include "hal/hal.h"
#include "hal/uart_print.h"
#include "hivemind_hal.h"
#include "lwip.h"

#ifdef IPERF_SERVER
#include <lwip/apps/lwiperf.h>
#endif

void Hal_init() {

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();

    MX_CRC_Init();

    MX_DMA_Init();
    MX_USART3_UART_Init();
    MX_USART2_UART_Init();
    MX_SPI5_Init();

    /* Initialize UartPrint */
    UartPrint_init();

    MX_LWIP_Init();

#ifdef IPERF_SERVER
    lwiperf_start_tcp_server_default(NULL, NULL);
#endif
}

uint32_t Hal_calculateCRC32(const uint8_t* buffer, uint32_t length) {
    // TODO: Determine what we do if we have a buffer that is not a multiple of 32 bytes
    // The HAL_CRC function takes the number of 32 bit words and not the number of bytes
    return HAL_CRC_Calculate(&hcrc, (uint32_t*)buffer, length >> 2);
}
