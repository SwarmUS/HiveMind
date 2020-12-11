#ifndef __FREERTOS_PLATFORM_CONFIG_H_
#define __FREERTOS_PLATFORM_CONFIG_H_

#include <stdint.h>
extern uint32_t SystemCoreClock;


/* Clock config that is platform dependent */
#define configCPU_CLOCK_HZ (SystemCoreClock)
#define configTICK_RATE_HZ ((TickType_t)1000)

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#endif // __FREERTOS_PLATFORM_CONFIG_H_
