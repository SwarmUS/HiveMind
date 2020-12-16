#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <task.h>
#include <timers.h>

#include <bsp/BSP.h>
#include <bsp/UserInterface.h>
#include <logger/Logger.h>

void printThreadExample(void* param) {
    (void)param;
    const int toggleDelay = 500;

    UserInterface ui = UserInterface();
    Logger logger = Logger(LogLevel::Debug, ui);

    logger.log(LogLevel::Info, "Hello logger!");
    while (true) {
        vTaskDelay(toggleDelay);
        ui.print("Hello world!");
    }
}

int main() {
    BSP bsp = BSP();
    bsp.initChip();

    xTaskCreate(printThreadExample, "print", configMINIMAL_STACK_SIZE * 4, NULL,
                tskIDLE_PRIORITY + 1, NULL);
    vTaskStartScheduler();
    while (true) {
    };

    return 0;
}
