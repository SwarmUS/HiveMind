#ifndef __TESTCHANNELS_H__
#define __TESTCHANNELS_H__

#include "IHardwareTest.h"
#include <deca_platform.h>

class TestChannels : public IHardwareTest {
  public:
    TestChannels() = default;
    virtual ~TestChannels() = default;

    void runTests() override;

  private:
    static void test01_Detect(decaDevice_t channel);
    static void test02_Enable(decaDevice_t channel);
    static void test03_Reset(decaDevice_t channel);
    void test04_IRQ(decaDevice_t channel);
    static void test05_SPI(decaDevice_t channel);
    static void test06_CS(decaDevice_t channel);

    static void isr(void* context);

    bool m_isrCalled = false;
};

#endif //__TESTCHANNELS_H__
