#include "hal/hal_timer.h"
#include "hal/user_interface.h"
#include "hivemind_hal.h"

static timerCallbackFct_t g_heartbeatCallback = UI_heartbeatCallback;

void Timer_startHeartbeat() { HAL_TIM_Base_Start_IT(HEARTBEAT_TIMER); }

void Timer_stopHeartbeat() { HAL_TIM_Base_Stop_IT(HEARTBEAT_TIMER); }

void Timer_setHeartbeatCallback(timerCallbackFct_t callback) { g_heartbeatCallback = callback; }

void CUSTOM_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == HEARTBEAT_TIMER) {
        if (g_heartbeatCallback != NULL) {
            g_heartbeatCallback();
        }
    }
}