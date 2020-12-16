#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <stdbool.h>
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
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
    // Using variables
    const void* taskName = pcTaskName;
    (void)xTask;

    taskDISABLE_INTERRUPTS();
    while (true) {
    };
}

#ifdef __cplusplus
}
#endif // __cplusplus
