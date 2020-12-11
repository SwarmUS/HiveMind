#include "bsp/bsp.h"

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <cstdio>
#include <task.h>
#include <timers.h>

void blinky(void* param) {
    while (true) {
        void* param2 = param;
        const int sleep = 500;
        printf("Light blinking\n");
        vTaskDelay(sleep);
    }
}

void init_chip() {}
