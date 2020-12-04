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
    while (true) {
    };
}

// NOLINTNEXTLINE(readability-non-const-parameter)
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char* pcTaskName) {
    // Using variables
    const void* taskName = pcTaskName;
    (void)pxTask;

    taskDISABLE_INTERRUPTS();
    while (true) {
    };
}

#ifdef __cplusplus
}
#endif // __cplusplus
