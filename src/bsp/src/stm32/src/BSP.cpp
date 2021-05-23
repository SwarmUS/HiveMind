#include "BSP.h"
#include "bsp/BSPContainer.h"
#include "bsp/SettingsContainer.h"
#include <hal/hal.h>
#include <logger/LoggerContainer.h>

BSP::BSP() : m_storage(LoggerContainer::getLogger()) {}
BSP::~BSP() = default;

void BSP::buttonCallback(void* context) {
    static_cast<BSP*>(context)->m_currentRGBState += 1;
    static_cast<BSP*>(context)->m_currentRGBState %= 8;
    BSPContainer::getUserInterface().setRGBLed(
        static_cast<RgbColor>(static_cast<BSP*>(context)->m_currentRGBState));
}

void BSP::initChip(void* args) {
    (void)args;

    Hal_init();
    m_storage.loadFromFlash();
    BSPContainer::getUserInterface().setButtonCallback(Button::BUTTON_0, buttonCallback, this);
    BSPContainer::getUserInterface().setRGBLed(static_cast<RgbColor>(m_currentRGBState));
}

uint16_t BSP::getUUId() const { return m_storage.getUUID(); }

uint32_t BSP::generateRandomNumber() { return Hal_generateRandomNumber(); }
