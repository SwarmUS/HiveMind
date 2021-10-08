#include "TestClock.h"
#include <deca_platform.h>
#include <deca_port.h>
#include <hal/hal_timer.h>
#include <hal/user_interface.h>
#include <hivemind_hal.h>

bool g_longPulse = false;
uint8_t g_counter = 0;

void TestClock::runTests() {
    UI_initialize();
    updateHex();
    UI_setButtonCallback(BUTTON_0, button0Callback, this);
    UI_setButtonCallback(BUTTON_1, button1Callback, this);

    startSquareWave();

    deca_init();

    while (true) {
        HAL_Delay(1000);
    }
}
void TestClock::startSquareWave() {
    HAL_TIM_Base_DeInit(HEARTBEAT_TIMER);

    // 1 kHZ wave
    HEARTBEAT_TIMER->Init.Prescaler = 2550;
    HEARTBEAT_TIMER->Init.Period = 50;
    if (HAL_TIM_Base_Init(HEARTBEAT_TIMER) != HAL_OK) {
        while (true) {
        }
    }

    Timer_startAll();
}

void TestClock::button0Callback(void* context) { static_cast<TestClock*>(context)->cycleMode(); }

void TestClock::button1Callback(void* context) {
    (void)context;
    pulseSync();
}

void TestClock::cycleMode() {
    m_testMode = (TestMode)(((uint8_t)m_testMode + 1) % (uint8_t)TestMode::NUM_MODES);
    if ((uint8_t)m_testMode == 0) {
        g_longPulse = !g_longPulse;
    }
    updateHex();

    switch (m_testMode) {
    case TestMode::SINGLE_PULSE:
        UI_setButtonCallback(BUTTON_1, button1Callback, this);
        Timer_setHeartbeatCallback(nullptr);
        break;

    case TestMode::MULTI_PUSLE:
        UI_setButtonCallback(BUTTON_1, nullptr, nullptr);
        Timer_setHeartbeatCallback(multiPulseTimerCallback);

        break;

    case TestMode::SQUARE_WAVE:
        UI_setButtonCallback(BUTTON_1, nullptr, nullptr);
        Timer_setHeartbeatCallback(sqareWaveTimerCallback);
        break;

    case TestMode::NUM_MODES:
        break;
    }
}

void TestClock::multiPulseTimerCallback() {
    g_counter++;

    if (g_counter == 19) {
        HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_SET);

        if (!g_longPulse) {
            HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);
            g_counter = 0;
        }
    } else if (g_counter >= 20) {
        HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);
        g_counter = 0;
    }
}
void TestClock::pulseSync() {
    HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_SET);
    if (g_longPulse) {
        HAL_Delay(1);
    }
    HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);
}

void TestClock::sqareWaveTimerCallback() { HAL_GPIO_TogglePin(SYNC_GPIO_Port, SYNC_Pin); }

void TestClock::updateHex() {
    uint8_t hex0 = (uint8_t)m_testMode;
    uint8_t hex1 = g_longPulse ? 1 : 0;
    UI_setHexOutput(hex1 << 4 | hex0);
}
