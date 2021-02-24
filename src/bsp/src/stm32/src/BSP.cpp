#include "BSP.h"
#include "bsp/SettingsContainer.h"
#include "deca_device_api.h"
#include <FreeRTOS.h>
#include <Task.h>
#include <hal/hal.h>

void decaBlink(void* params) {
    (void)params;
    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR) {
        while (true)
            ;
    }

    dwt_enablegpioclocks();

    while (true) {
        dwt_setgpiovalue(DWT_GxM2, 0);

        Task::delay(250);

        dwt_setgpiovalue(DWT_GxM2, 1);

        Task::delay(250);
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
