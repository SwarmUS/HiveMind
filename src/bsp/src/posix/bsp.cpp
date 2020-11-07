#include "bsp.h"

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <task.h>
#include <timers.h>
#include <stdio.h>

void blinky() {
  for (;;) {
    vTaskDelay(500);
    printf("WOW");
  }
}

void initGPIO() {

}

void init_chip() {
}
