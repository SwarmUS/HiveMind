#include "BSP.h"
#include "bsp/SettingsContainer.h"
#include "deca_device_api.h"
#include "deca_port.h"
#include <Decawave.h>
#include <FreeRTOS.h>
#include <Task.h>
#include <hal/hal.h>

// TODO: Only here as example. Move everything out to a Decawave class
void decaBlink(void* params) {
    (void)params;

    deca_selectDevice(DW_A);
    while (DWT_DEVICE_ID != dwt_readdevid()) {
        Task::delay(1);
    }
    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR) {
        while (true) {
        }
    }

    deca_selectDevice(DW_B);
    while (DWT_DEVICE_ID != dwt_readdevid()) {
        Task::delay(1);
    }
    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR) {
        while (true) {
        }
    }

    deca_selectDevice(DW_A);
    dwt_enablegpioclocks();
    dwt_setgpiodirection(DWT_GxM3 | DWT_GxM0 | DWT_GxM1 | DWT_GxM2 | DWT_GxM6 | DWT_GxM5,
                         DWT_GxP6 | DWT_GxP5);

    deca_selectDevice(DW_B);
    dwt_enablegpioclocks();
    dwt_setgpiodirection(DWT_GxM3 | DWT_GxM0 | DWT_GxM1 | DWT_GxM2 | DWT_GxM6 | DWT_GxM5,
                         DWT_GxP6 | DWT_GxP5);
    Task::delay(100);

    while (true) {
        deca_selectDevice(DW_B);
        dwt_setgpiovalue(DWT_GxM0, DWT_GxP0);
        Task::delay(100);
        dwt_setgpiovalue(DWT_GxM1, DWT_GxP1);
        Task::delay(100);
        dwt_setgpiovalue(DWT_GxM2, DWT_GxP2);
        Task::delay(100);
        dwt_setgpiovalue(DWT_GxM3, DWT_GxP3);

        Task::delay(100);
        dwt_setgpiovalue(DWT_GxM0, 0);
        deca_selectDevice(DW_A);
        dwt_setgpiovalue(DWT_GxM0, DWT_GxP0);

        Task::delay(100);
        deca_selectDevice(DW_B);
        dwt_setgpiovalue(DWT_GxM1, 0);
        deca_selectDevice(DW_A);
        dwt_setgpiovalue(DWT_GxM1, DWT_GxP1);

        Task::delay(100);
        deca_selectDevice(DW_B);
        dwt_setgpiovalue(DWT_GxM2, 0);
        deca_selectDevice(DW_A);
        dwt_setgpiovalue(DWT_GxM2, DWT_GxP2);

        Task::delay(100);
        deca_selectDevice(DW_B);
        dwt_setgpiovalue(DWT_GxM3, 0);
        deca_selectDevice(DW_A);
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

    // m_decaBlinkTask.start();
    Decawave deca = Decawave(DW_A, 2, UWBSpeed::SPEED_110K);
    deca.start();

    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};

    while (true) {
        deca.transmit(data, sizeof data);
        Task::delay(1000);
    }
}

uint16_t BSP::getUUId() const { return SettingsContainer::getUUID(); }

uint32_t BSP::generateRandomNumber() { return Hal_generateRandomNumber(); }
