#ifndef HIVE_CONNECT_SPISTM_H
#define HIVE_CONNECT_SPISTM_H

#include "bsp/ISpiEsp.h"

class SpiEspMock : public ISpiEsp {
  public:
    SpiEspMock() = default;
    ~SpiEspMock() override = default;

    bool send(const uint8_t* buffer, uint16_t length) override {
        (void)buffer;
        (void)length;
        return false;
    };

    bool isBusy() const override { return false; }
};

#endif // HIVE_CONNECT_SPISTM_H
