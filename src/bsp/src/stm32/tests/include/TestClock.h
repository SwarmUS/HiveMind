#ifndef __TESTCLOCK_H__
#define __TESTCLOCK_H__

#include "IHardwareTest.h"

class TestClock : public IHardwareTest {
  public:
    enum class TestMode { SINGLE_PULSE, MULTI_PUSLE, SQUARE_WAVE, NUM_MODES };

    TestClock() = default;
    ~TestClock() = default;

    void runTests() override;

  private:
    TestMode m_testMode = TestMode::SINGLE_PULSE;

    void updateHex();

    static void startSquareWave();
    void cycleMode();

    static void button0Callback(void* context);
    static void button1Callback(void* context);

    static void multiPulseTimerCallback();
    static void sqareWaveTimerCallback();
    static void pulseSync();
};

#endif //__TESTCLOCK_H__
