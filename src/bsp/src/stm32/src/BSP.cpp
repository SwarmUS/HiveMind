#include "BSP.h"
#include "bsp/SettingsContainer.h"
#include "deca_device_api.h"
#include <FreeRTOS.h>
#include <Task.h>
#include <hal/hal.h>

void decaBlink(void* params) {
    (void)params;
    while (DWT_DEVICE_ID != dwt_readdevid()) {
        Task::delay(1);
    }
    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR) {
        while (true) {
        }
    }

    dwt_enablegpioclocks();
    dwt_setgpiodirection(DWT_GxM3 | DWT_GxM0 | DWT_GxM1 | DWT_GxM2 | DWT_GxM6 | DWT_GxM5,
                         DWT_GxP6 | DWT_GxP5);
    Task::delay(100);

    while (true) {
        dwt_setgpiovalue(DWT_GxM0, DWT_GxP0);
        Task::delay(100);
        dwt_setgpiovalue(DWT_GxM1, DWT_GxP1);
        Task::delay(100);
        dwt_setgpiovalue(DWT_GxM2, DWT_GxP2);
        Task::delay(100);
        dwt_setgpiovalue(DWT_GxM3, DWT_GxP3);

        Task::delay(100);
        dwt_setgpiovalue(DWT_GxM0, 0);
        Task::delay(100);
        dwt_setgpiovalue(DWT_GxM1, 0);
        Task::delay(100);
        dwt_setgpiovalue(DWT_GxM2, 0);
        Task::delay(100);
        dwt_setgpiovalue(DWT_GxM3, 0);
    }
}

BSP::BSP() : m_decaBlinkTask("deca_blink", tskIDLE_PRIORITY + 1, decaBlink, NULL) {}
BSP::~BSP() = default;

void BSP::initChip(void* args) {
    (void)args;

    Hal_init();

    m_decaBlinkTask.start();
}

uint16_t BSP::getUUId() const { return SettingsContainer::getUUID(); }

uint32_t BSP::generateRandomNumber() { return Hal_generateRandomNumber(); }
