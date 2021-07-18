#ifndef __TESTUSB_H__
#define __TESTUSB_H__

#include "IHardwareTest.h"
#include <stdint.h>

class TestUSB : public IHardwareTest {
  public:
    TestUSB() = default;
    ~TestUSB() = default;

    void runTests() override;

  private:
    static void setup();

    static void test01_UART();
    void test02_USB();

    static void rxCallback(void* context, uint8_t* buffer, uint32_t length);
    bool m_receivedData = false;
};

#endif //__TESTUSB_H__
