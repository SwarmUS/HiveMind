#include "bsp/bsp.h"

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <cstdio>
#include <cstdlib>
#include <task.h>
#include <timers.h>

void blinky(void* ptr) {
    while (true) {
        const int sleep = 500;
        printf("Light blinking\n");
        vTaskDelay(sleep);
    }
}

void init_chip() {}
