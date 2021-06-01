#include "hal/hal_timer.h"
#include "hal/user_interface.h"
#include "hivemind_hal.h"

void Timer_startHeartbeat() { HAL_TIM_Base_Start_IT(HEARTBEAT_TIMER); }

void CUSTOM_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == HEARTBEAT_TIMER) {
        UI_heartbeatCallback();
    }
}