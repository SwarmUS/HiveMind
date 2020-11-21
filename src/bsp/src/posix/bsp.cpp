#include "bsp/bsp.h"

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <stdio.h>
#include <task.h>
#include <timers.h>

void blinky(void *ptr) {
  while (1) {
    printf("Light blinking\n");
    vTaskDelay(500);
  }
}

void init_chip() {}
