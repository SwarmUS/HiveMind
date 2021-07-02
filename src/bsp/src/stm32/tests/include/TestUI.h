#ifndef __TESTUI_H__
#define __TESTUI_H__

#include "IHardwareTest.h"

class TestUI : public IHardwareTest {
  public:
    TestUI() = default;
    ~TestUI() = default;

    void runTests() override;

  private:
    static void setup();

    static void test01_Heartbeat();
    static void test02_Leds();
    static void test03_RGB();
    void test04_Buttons();
    static void test05_HexDisplay();

    bool m_button0Pressed = false;
    bool m_button1Pressed = false;

    static void button0Callback(void* context);
    static void button1Callback(void* context);
};

#endif //__TESTUI_H__
