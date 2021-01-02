#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <task.h>
#include <timers.h>

#include <bittybuzz/BittyBuzzBytecode.h>
#include <bittybuzz/BittyBuzzFactory.h>
#include <bittybuzz/BittyBuzzVm.h>
#include <bsp/BSP.h>
#include <bsp/UserInterface.h>
#include <cstdlib>
#include <logger/Logger.h>

void printThreadExample(void* param) {
    (void)param;
    const int toggleDelay = 2;

    BSP bsp = BSP();
    bsp.initChip();

    UserInterface ui = UserInterface();
    Logger logger = Logger(LogLevel::Debug, ui);

    BittyBuzzBytecode bytecode = BittyBuzzFactory::createBittyBuzzBytecode(logger);
    std::array<FunctionRegister, 1> functionRegisters =
        BittyBuzzFactory::createBittyBuzzFunctionRegisters();
    BittyBuzzVm bittybuzz = BittyBuzzVm(bytecode, bsp, logger, functionRegisters);

    logger.log(LogLevel::Info, "Hello logger!");
    while (true) {
        logger.log(LogLevel::Info, "Hello world!");
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
