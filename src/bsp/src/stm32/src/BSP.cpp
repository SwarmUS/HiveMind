#include "BSP.h"
#include "bsp/SettingsContainer.h"
#include <Decawave.h>
#include <Task.h>
#include <hal/hal.h>

BSP::BSP() = default;
BSP::~BSP() = default;

void BSP::initChip(void* args) {
    (void)args;

    Hal_init();
}

void BSP::deca() const {
    Decawave decaA = Decawave(DW_A, 2, UWBSpeed::SPEED_110K);
    decaA.start();

    Decawave decaB = Decawave(DW_B, 2, UWBSpeed::SPEED_110K);
    decaB.start();

    uint8_t data[] = {0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
    UWBRxFrame rxFrame;
    (void)data;

    while (true) {
        decaB.transmitAndReceive(data, sizeof(data), 1, rxFrame, 0);
        // decaA.transmit(data, sizeof(data));

        while (rxFrame.m_status == UWBRxStatus::ONGOING) {
        }
        Task::delay(1000);
    }
}

uint16_t BSP::getUUId() const { return SettingsContainer::getUUID(); }

uint32_t BSP::generateRandomNumber() { return Hal_generateRandomNumber(); }
