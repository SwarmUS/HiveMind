#include <stdio.h>
#include <time.h>

void vApplicationIdleHook(void) {
    // This task is run when FreeRTOS has nothing to do. Add sleep to prevent using 100% of system
    // CPU. The sleep doesn't block other tasks from executing so its length doesnt matter.
    // https://www.freertos.org/FreeRTOS-simulator-for-Linux.html#known_issues
    const int sleepSec = 5;
    const int sleepNano = 0;

    nanosleep((const struct timespec[]){{sleepSec, sleepNano}}, NULL);
}
