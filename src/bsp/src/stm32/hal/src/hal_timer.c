#include "hal/hal_timer.h"
#include "hal/user_interface.h"
#include "hivemind_hal.h"

static timerCallbackFct_t g_heartbeatCallback = UI_heartbeatCallback;
static timerCallbackFct_t g_hundredMicrosCallback = NULL;
static void* g_hundredMicrosContext = NULL;

void Timer_startAll() {
    HAL_TIM_Base_Start_IT(HEARTBEAT_TIMER);
    HAL_TIM_Base_Start_IT(HUNDREDMICROSECONDS_TIMER);
}

void Timer_stopHeartbeat() { HAL_TIM_Base_Stop_IT(HEARTBEAT_TIMER); }

void Timer_setHeartbeatCallback(timerCallbackFct_t callback) { g_heartbeatCallback = callback; }

void Timer_setHundredMicrosCallback(timerCallbackFct_t callback, void* context) {
    g_hundredMicrosCallback = callback;
    g_hundredMicrosContext = context;
}

void CUSTOM_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == HEARTBEAT_TIMER) {
        if (g_heartbeatCallback != NULL) {
            g_heartbeatCallback(NULL);
        }
    } else if (htim == HUNDREDMICROSECONDS_TIMER) {
        if (g_hundredMicrosCallback != NULL) {
            g_hundredMicrosCallback(g_hundredMicrosContext);
        }
    }
}