#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <task.h>
#include <timers.h>

#include <bsp/bsp.h>

int main(void) {
  init_chip();
  xTaskCreate(blinky, "blinky", configMINIMAL_STACK_SIZE * 4, NULL,
              tskIDLE_PRIORITY + 1, NULL);
  vTaskStartScheduler();
  while (1)
    ;

  return 0;
}
