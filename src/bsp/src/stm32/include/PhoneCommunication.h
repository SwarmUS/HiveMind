#ifndef __PHONECOMMUNICATION_H__
#define __PHONECOMMUNICATION_H__

#define PHONE_COMMUNICATION_MAX_LENGTH 65536

#include "bsp/IPhoneCommunication.h"

class PhoneCommunication : public IPhoneCommunication {
  public:
    PhoneCommunication() = default;
    ~PhoneCommunication() = default;

    void sendBytes(const uint8_t* bytes, uint16_t length) override;
    void registerCallback() override;

    bool isBusy() override;

  private:
    uint8_t txBuffer[PHONE_COMMUNICATION_MAX_LENGTH];
    uint8_t rxBuffer[PHONE_COMMUNICATION_MAX_LENGTH];
};

#endif //__PHONECOMMUNICATION_H__
