#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <task.h>
#include <timers.h>

#include <bittybuzz/BittyBuzzBytecode.h>
#include <bittybuzz/BittyBuzzVm.h>
#include <bsp/BSP.h>
#include <bsp/UserInterface.h>
#include <cstdlib>
#include <logger/Logger.h>

extern "C" {
#include <main_bytecode.h>
}


void printThreadExample(void* param) {
    (void)param;
    const int toggleDelay = 2;

    BSP bsp = BSP();
    bsp.initChip();

    UserInterface ui = UserInterface();
    Logger logger = Logger(LogLevel::Debug, ui);

    BittyBuzzBytecode bytecode(logger, bcode, bcode_size);
    BittyBuzzVm bittybuzz = BittyBuzzVm(bytecode, bsp, logger);

    logger.log(LogLevel::Info, "Hello logger!");
    while (true) {
        logger.log(LogLevel::Info, "Hello world!");
        logger.log(LogLevel::Info, "Hello world! TEST %d", 1);
        vTaskDelay(toggleDelay);
        bittybuzz.step();
    }
}

int main() {

    xTaskCreate(printThreadExample, "print", configMINIMAL_STACK_SIZE * 4, NULL,
                tskIDLE_PRIORITY + 1, NULL);
    vTaskStartScheduler();
    while (true) {
    };

    return 0;
}
