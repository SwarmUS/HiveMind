#include "TestClock.h"
#include <deca_platform.h>
#include <hal/hal_timer.h>
#include <hivemind_hal.h>

void TestClock::runTests() {
    setupSyncSquareWave();
    startSquareWave();

    for (int i = 0; i < 7; i++) {
        decaDevice_t channel = (decaDevice_t)i;

        // Step 2.0
        beeboard_enableClock(channel);
        beeboard_enableChannel(channel);

        // Step 2.1
        beeboard_disableChannel(channel);

        // Step 2.2
        beeboard_enableChannel(channel);
        beeboard_disableClock(channel);

        // Cleanup
        beeboard_disableChannel(channel);
        beeboard_disableClock(channel);
    }
}

void TestClock::clockQualification() {}

void TestClock::heartbeatCallback() { HAL_GPIO_TogglePin(SYNC_GPIO_Port, SYNC_Pin); }

void TestClock::setupSyncSquareWave() {
    HAL_TIM_Base_DeInit(HEARTBEAT_TIMER);

    Timer_setHeartbeatCallback(heartbeatCallback);

    // 520 MHZ base clock / 5200 prescaler / 50 Period = 2 kHz toggle period = 1 KHz wave
    HEARTBEAT_TIMER->Init.Prescaler = 5200;
    HEARTBEAT_TIMER->Init.Period = 50;
    if (HAL_TIM_Base_Init(HEARTBEAT_TIMER) != HAL_OK) {
        while (true) {
        }
    }
}

void TestClock::startSquareWave() { Timer_startHeartbeat(); }

void TestClock::stopSquareWave() { Timer_stopHeartbeat(); }
