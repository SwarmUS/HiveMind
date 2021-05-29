#include "hal/hal_timer.h"
#include "hal/user_interface.h"
#include "hivemind_hal.h"

void USER_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == HEARTBEAT_TIMER) {
        UI_heartbeatCallback();
    }
}