#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <task.h>

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif // __cplusplus
