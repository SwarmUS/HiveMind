#include "TestNetwork.h"
#include <Task.h>
#include <hal/hal_init.h>
#include <lwip.h>
#include <lwip/apps/lwiperf.h>

void TestNetwork::runTests() {
    Hal_enableEthernet();

    MX_LWIP_Init();
    lwiperf_start_tcp_server_default(NULL, NULL);
    Task::startScheduler();
}
