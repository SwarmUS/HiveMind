
#ifndef HIVE_MIND_USBMOCK_H
#define HIVE_MIND_USBMOCK_H

class USBMock : public IUSB {
  public:
    ~USBMock() override = default;

    bool send(const uint8_t* buffer, uint16_t length) override { return true; }
    bool receive(uint8_t* buffer, uint16_t length) override { return true; }
    bool isConnected() override { return true; }
};

#endif // HIVE_MIND_USBMOCK_H
