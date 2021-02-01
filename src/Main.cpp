#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <task.h>
#include <timers.h>

#include <bittybuzz/BittyBuzzFactory.h>
#include <bittybuzz/BittyBuzzVm.h>
#include <bsp/BSPContainer.h>
#include <bsp/IBSP.h>
#include <bsp/SocketContainer.h>
#include <cstdlib>
#include <logger/Logger.h>
#include <logger/LoggerContainer.h>

void printThreadExample(void* param) {
    (void)param;
    const int toggleDelay = 2000;

    ILogger& logger = LoggerContainer::getLogger();

    std::optional<TCPClientWrapper> socket = SocketContainer::getHostClientSocket();

    if (socket) {
        socket.value().send((const uint8_t*)"HELLO WORLD", sizeof("HELLO WORD"));
    }

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

        IHostUart& hostUart = BSPContainer::getHostUart();
        hostUart.send((const uint8_t*)"HELLO WORLD", sizeof("HELLO WORLD"));
    }
}

int main(int argc, char** argv) {
    CmdLineArgs cmdLineArgs = {argc, argv};

    IBSP& bsp = BSPContainer::getBSP();
    bsp.initChip((void*)&cmdLineArgs);

    const uint32_t stackSize =  configMINIMAL_STACK_SIZE * 8;
    xTaskCreate(printThreadExample, "print", stackSize, NULL,
                tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();

    return 0;
}
