#include "hal/hal.h"
#include "deca_port.h"
#include "hal/hal_gpio.h"
#include "hal/hal_init.h"
#include "hal/hal_timer.h"
#include "hal/uart_print.h"
#include "hal/usb.h"
#include "hal/user_interface.h"
#include "hivemind_hal.h"
#include "lwip.h"
#include "usb_device.h"

#ifdef IPERF_SERVER
#include <lwip/apps/lwiperf.h>
#endif

#ifdef RUNTIME_STATS
uint32_t BSP_getCPUCounter(){
    return RUNTIME_STATS_TIMER->Instance->CNT;
}
#endif // RUNTIME_STATS

void Hal_initMcu() {

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    MX_RNG_Init();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();

    MX_CRC_Init();
    MX_DMA_Init();

    PHal_initMcu();
    UartPrint_init();
}

void Hal_initBoard() {
    deca_init();

    UI_initialize();
    Timer_startHeartbeat();

    if (Hal_wroomPowerEnabled()) {
        Hal_enableWroom();
    }

    if (Hal_ethernetPowerEnabled()) {
        Hal_enableEthernet();
        MX_LWIP_Init();
#ifdef IPERF_SERVER
        lwiperf_start_tcp_server_default(NULL, NULL);
#endif
    }
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
