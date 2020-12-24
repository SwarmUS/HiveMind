#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <task.h>
#include <timers.h>

#include <bsp/BSP.h>
#include <bsp/UserInterface.h>
#include <cstdlib>
#include <logger/Logger.h>
#include <bittybuzz/BittyBuzz.h>

void printThreadExample(void* param) {
    (void)param;
    const int toggleDelay = 2;

    UserInterface ui = UserInterface();
    Logger logger = Logger(LogLevel::Debug, ui);

    BittyBuzz bittybuzz = BittyBuzz();
    bittybuzz.init();
    bittybuzz.run();

    logger.log(LogLevel::Info, "Hello logger!");
    while (true) {
        ui.print("Hello world!");
        vTaskDelay(toggleDelay);
        ui.print("Goodbye!");
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
