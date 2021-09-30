#ifndef __FREERTOS_PLATFORM_CONFIG_H_
#define __FREERTOS_PLATFORM_CONFIG_H_

#include <stdint.h>
extern uint32_t SystemCoreClock;

/*
 * The CMSIS-RTOS V2 FreeRTOS wrapper is dependent on the heap implementation used
 * by the application thus the correct define need to be enabled below (only used if heap 1 or 5)
 */
#define USE_FreeRTOS_HEAP_3

/* Clock config that is platform dependent */
#define configCPU_CLOCK_HZ (SystemCoreClock)
#define configTICK_RATE_HZ ((TickType_t)100)

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#endif // __FREERTOS_PLATFORM_CONFIG_H_
