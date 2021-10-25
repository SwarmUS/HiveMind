#include "BSP.h"
#include "UserInterface.h"
#include "bsp/BSPContainer.h"
#include <Task.h>
#include <hal/hal.h>
#include <logger/LoggerContainer.h>

#ifdef RUNTIME_STATS
#include "task.h"
static char buffer[1024];
void usageFunctionCallback(void* ctx) {
    (void)ctx;
    ILogger& logger = LoggerContainer::getLogger();

    while (1) {
        vTaskGetRunTimeStats(buffer);
        logger.log(LogLevel::Info, "\n%s", buffer);
        Task::delay(10000); // 10 sec
    }
}
#endif // RUNTIME_STATS

BSP::BSP() : m_storage(LoggerContainer::getLogger()) {}
BSP::~BSP() = default;

// Example button callback that cycles through the RGB possibilities
void BSP::buttonCallback(void* context) {
    static_cast<BSP*>(context)->m_currentRGBState += 1;
    static_cast<BSP*>(context)->m_currentRGBState %= 8;
    reinterpret_cast<UserInterface&>(BSPContainer::getUserInterface())
        .setRGBLed(static_cast<RgbColor>(static_cast<BSP*>(context)->m_currentRGBState));
}

void BSP::initChip(void* args) {
    (void)args;

    Hal_initMcu();
    Hal_initBoard();

    m_storage.loadFromFlash();

#ifdef RUNTIME_STATS
    TaskHandle_t xHandle = NULL;
    xTaskCreate(usageFunctionCallback, "cpu-usage", 4 * configMINIMAL_STACK_SIZE, NULL,
                tskIDLE_PRIORITY + 1, &xHandle);
#endif // RUNTIME_STATS
}

uint16_t BSP::getUUId() const { return m_storage.getUUID(); }

uint32_t BSP::generateRandomNumber() { return Hal_generateRandomNumber(); }

PersistantStorageManager& BSP::getStorage() { return m_storage; }
