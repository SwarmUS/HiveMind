#ifndef __TESTNEWORK_H__
#define __TESTNEWORK_H__

#include "IHardwareTest.h"
#include <stdint.h>

class TestNetwork : public IHardwareTest {
  public:
    TestNetwork() = default;
    ~TestNetwork() = default;

    void runTests() override;
};

#endif //__TESTNEWORK_H__
