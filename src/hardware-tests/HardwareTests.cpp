#include "../bsp/src/stm32/tests/include/TestChannels.h"
#include "../bsp/src/stm32/tests/include/TestClock.h"
#include "../bsp/src/stm32/tests/include/TestNetwork.h"
#include "../bsp/src/stm32/tests/include/TestUI.h"
#include "../bsp/src/stm32/tests/include/TestUSB.h"
#include <bsp/BSPContainer.h>
#include <bsp/IBSP.h>

int main(int argc, char** argv) {
    CmdLineArgs cmdLineArgs = {argc, argv};

    IBSP& bsp = BSPContainer::getBSP();
    bsp.initChip((void*)&cmdLineArgs);

    static TestUI s_testUI;
    static TestUSB s_testUsb;
    static TestChannels s_testChannels;
    static TestNetwork s_testNetwork;
    static TestClock s_testClock;

    s_testUI.runTests();
    s_testUsb.runTests();
    s_testChannels.runTests();
    s_testClock.runTests();

    // The network tests start the FreeRTOS scheduler and are therefore blocking.
    // They should be run last
    s_testNetwork.runTests();

    return 0;
}