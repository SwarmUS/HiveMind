#include <TestChannels.h>
#include <TestClock.h>
#include <TestNetwork.h>
#include <TestUI.h>
#include <TestUSB.h>
#include <hal/hal.h>

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    Hal_initMcu();

    static TestUI s_testUI;
    static TestUSB s_testUsb;
    static TestChannels s_testChannels;
    static TestNetwork s_testNetwork;
    static TestClock s_testClock;

    // *************
    // Activate the test you want to run by uncommenting the appropriate line
    // *************

    s_testUI.runTests();
    // s_testUsb.runTests();
    // s_testChannels.runTests();
    // s_testClock.runTests();

    // The network tests start the FreeRTOS scheduler and are therefore blocking.
    // They should be run last
    // s_testNetwork.runTests();

    return 0;
}