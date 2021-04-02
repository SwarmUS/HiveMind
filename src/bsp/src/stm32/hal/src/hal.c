#include "hal/hal.h"
#include "deca_port.h"
#include "hal/uart_print.h"
#include "hal/usb.h"
#include "hivemind_hal.h"
#include "lwip.h"
#include "usb_device.h"

#ifdef IPERF_SERVER
#include <lwip/apps/lwiperf.h>
#endif

void Hal_init() {

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    HAL_RNG_Init(HRNG);

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();

    MX_CRC_Init();

    MX_DMA_Init();
    MX_USART3_UART_Init();
    MX_SPI3_Init();
    MX_SPI5_Init();
    MX_SPI4_Init();
    MX_USB_DEVICE_Init();

    UartPrint_init();
    usb_init();

    MX_LWIP_Init();

    deca_init();
#ifdef IPERF_SERVER
    lwiperf_start_tcp_server_default(NULL, NULL);
#endif
}

uint32_t Hal_calculateCRC32(const uint8_t* buffer, uint32_t length) {
    // TODO: Determine what we do if we have a buffer that is not a multiple of 32 bytes
    // The HAL_CRC function takes the number of 32 bit words and not the number of bytes
    return HAL_CRC_Calculate(&hcrc, (uint32_t*)buffer, length >> 2);
}

uint32_t Hal_generateRandomNumber() {
    uint32_t random = 0;
    // TODO: Error handling if the generation fails
    HAL_RNG_GenerateRandomNumber(HRNG, &random);
    return random;
}
