#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <task.h>
#include <timers.h>

#include <bittybuzz/BittyBuzzFactory.h>
#include <bittybuzz/BittyBuzzVm.h>
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

    BittyBuzzBytecode bytecode = BittyBuzzFactory::createBittyBuzzBytecode(logger);
    BittyBuzzStringResolver stringResolver =
        BittyBuzzFactory::createBittyBuzzStringResolver(logger);

    auto functionRegisters = BittyBuzzFactory::createBittyBuzzFunctionRegisters();

    BittyBuzzVm bittybuzz =
        BittyBuzzVm(bytecode, stringResolver, BSPContainer::getBSP(), logger, functionRegisters);

    logger.log(LogLevel::Info, "Hello logger!");
    while (true) {
        logger.log(LogLevel::Info, "Hello world!");
        vTaskDelay(toggleDelay);
        bittybuzz.step();
    }
}

int main(int argc, char** argv) {
    CmdLineArgs cmdLineArgs = {argc, argv};

    IBSP& bsp = BSPContainer::getBSP();
    bsp.initChip((void*)&cmdLineArgs);

    xTaskCreate(printThreadExample, "print", configMINIMAL_STACK_SIZE * 4, NULL,
                tskIDLE_PRIORITY + 1, NULL);
    vTaskStartScheduler();

    return 0;
}
