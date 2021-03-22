
#ifndef HIVE_MIND_USBMOCK_H
#define HIVE_MIND_USBMOCK_H

#include "bsp/IUSB.h"

class USBMock : public IUSB {
  public:
    ~USBMock() override = default;

    bool send(const uint8_t* buffer, uint16_t length) override {
        (void)buffer;
        (void)length;

        return true;
    }
    bool receive(uint8_t* buffer, uint16_t length) override {
        (void)buffer;
        (void)length;

        return true;
    }
    bool isConnected() const override { return true; }
};

#endif // HIVE_MIND_USBMOCK_H
