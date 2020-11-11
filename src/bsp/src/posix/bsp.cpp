#include "bsp/bsp.h"

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <stdio.h>
#include <task.h>
#include <timers.h>

void blinky() {
  while (1) {
    printf("Light blinking\n");
    vTaskDelay(500);
  }
}

void initGPIO() {}

void init_chip() {}
