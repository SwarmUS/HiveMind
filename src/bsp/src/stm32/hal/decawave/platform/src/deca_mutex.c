#include "deca_device_api.h"
#include <FreeRTOS.h>

// This file is used by Decawave API to disable interrupts. Because we are using FreeRTOS. The state
// saving mechanism is already handled by the OS. We can only enter/exit a critical code section.

decaIrqStatus_t decamutexon(void) {
    if (!xPortIsInsideInterrupt()) {
        vPortEnterCritical();
    }
    return 0;
}

void decamutexoff(decaIrqStatus_t s) {
    (void)s;

    if (!xPortIsInsideInterrupt()) {
        vPortExitCritical();
    }
}
