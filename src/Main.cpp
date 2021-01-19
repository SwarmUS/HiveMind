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
        uint8_t bytes[] = {0x01, 0x02, 0x03, 0x04};
        hostUart.sendBytes(bytes, 4);
    }
}

int main(int argc, char** argv) {
    CmdLineArgs cmdLineArgs = {argc, argv};

    IBSP& bsp = BSPContainer::getBSP();
    bsp.initChip((void*)&cmdLineArgs);
    IHostUart& hostUart = BSPContainer::getHostUart();
    (void)hostUart;

    xTaskCreate(printThreadExample, "print", configMINIMAL_STACK_SIZE * 8, NULL,
                tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();

    return 0;
}
