#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <task.h>
#include <timers.h>

#include <bsp.h>

#include <stdio.h>

int main(void) {
  xTaskCreate(blinky, "blinky", configMINIMAL_STACK_SIZE * 4, NULL,
              tskIDLE_PRIORITY + 1, NULL);
  vTaskStartScheduler();
  while (1)
    ;

  return 0;
}

void vApplicationTickHook(void) {}

void vApplicationIdleHook(void) {}

void vApplicationMallocFailedHook(void) {
  taskDISABLE_INTERRUPTS();
  while (1)
    ;
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
  (void)pcTaskName;
  (void)pxTask;

  taskDISABLE_INTERRUPTS();
  while (1)
    ;
}
