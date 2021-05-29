#ifndef __HAL_TIMER_H__
#define __HAL_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"

/**
 * @brief Starts the heartbeat timer interrupt
 */
void Timer_startHeartbeat();

#ifdef __cplusplus
}
#endif

#endif //__HAL_TIMER_H__
