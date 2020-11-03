#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <task.h>
#include <timers.h>

#include <bsp.h>

int main(void) {

  int test = BSP_TEST;
  blinky();
  vTaskStartScheduler();
  for (;;)
    ;

  return 0;
}

void vApplicationTickHook(void) {}

void vApplicationIdleHook(void) {}

void vApplicationMallocFailedHook(void) {
  taskDISABLE_INTERRUPTS();
  for (;;)
    ;
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
  (void)pcTaskName;
  (void)pxTask;

  taskDISABLE_INTERRUPTS();
  for (;;)
    ;
}
