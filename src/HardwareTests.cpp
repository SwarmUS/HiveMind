#include <bsp/BSPContainer.h>
#include <bsp/IBSP.h>
#include <bsp/src/stm32/tests/include/TestChannels.h>
#include <bsp/src/stm32/tests/include/TestUI.h>
#include <bsp/src/stm32/tests/include/TestUSB.h>

int main(int argc, char** argv) {
    CmdLineArgs cmdLineArgs = {argc, argv};

    IBSP& bsp = BSPContainer::getBSP();
    bsp.initChip((void*)&cmdLineArgs);

    static TestUI s_testUi;
    static TestUSB s_testUsb;
    static TestChannels s_testChannels;

    s_testUi.runTests();
    s_testUsb.runTests();
    s_testChannels.runTests();

    return 0;
}