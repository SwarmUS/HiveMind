#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <task.h>
#include <timers.h>

#include <bsp/BSPContainer.h>
#include <bsp/IBSP.h>
#include <bsp/UserInterface.h>
#include <cstdlib>
#include <logger/Logger.h>

void printThreadExample(void* param) {
    (void)param;
    const int toggleDelay = 2000;

    UserInterface ui = UserInterface();
    Logger logger = Logger(LogLevel::Debug, ui);

    logger.log(LogLevel::Info, "Hello logger!");
    while (true) {
        ui.print("Hello world!");
        vTaskDelay(toggleDelay);
        ui.print("Goodbye!");
    }
}

int main(int argc, char** argv) {
    CmdLineArgs cmdLineArgs = {argc, argv};

    IBSP* bsp = BSPContainer::getBSP();
    bsp->initChip((void*)&cmdLineArgs);

    xTaskCreate(printThreadExample, "print", configMINIMAL_STACK_SIZE * 4, NULL,
                tskIDLE_PRIORITY + 1, NULL);
    vTaskStartScheduler();

    return 0;
}
