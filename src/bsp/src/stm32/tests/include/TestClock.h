#ifndef __TESTCLOCK_H__
#define __TESTCLOCK_H__

#include "IHardwareTest.h"

class TestClock : public IHardwareTest {
  public:
    TestClock() = default;
    ~TestClock() = default;

    void runTests() override;
    static void clockQualification();

  private:
    bool m_buttonPressed = false;
    void waitButtonPress();
    static void buttonCallback(void* context);

    static void startSquareWave();
    static void stopSquareWave();
    static void setupSyncSquareWave();
    static void heartbeatCallback();
};

#endif //__TESTCLOCK_H__
