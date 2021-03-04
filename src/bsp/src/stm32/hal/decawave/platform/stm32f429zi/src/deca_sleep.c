#include "deca_device_api.h"
#include "deca_port.h"
#include <FreeRTOS.h>
#include <task.h>

/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
__INLINE void deca_sleep(unsigned int time_ms) { vTaskDelay(time_ms / portTICK_PERIOD_MS); }
