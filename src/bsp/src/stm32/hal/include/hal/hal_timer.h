#ifndef __HAL_TIMER_H__
#define __HAL_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"

typedef void (*timerCallbackFct_t)(void*);

/**
 * @brief Starts the heartbeat timer interrupt
 */
void Timer_startAll();

/**
 * @brief Stops the heartbeat timer interrupt
 */
void Timer_stopHeartbeat();

/**
 * @brief Sets the callback to be called when a timer period elapses
 * @param callback Function to call
 */
void Timer_setHeartbeatCallback(timerCallbackFct_t callback);

void Timer_setHundredMicrosCallback(timerCallbackFct_t callback, void* context);

#ifdef __cplusplus
}
#endif

#endif //__HAL_TIMER_H__
