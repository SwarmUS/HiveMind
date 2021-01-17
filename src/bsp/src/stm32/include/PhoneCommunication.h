#ifndef __PHONECOMMUNICATION_H__
#define __PHONECOMMUNICATION_H__

#define PHONE_COMMUNICATION_MAX_LENGTH 65536
#define PHONE_COMMUNICATION_HEADER_LENGTH 6

#include "bsp/IPhoneCommunication.h"
#include <FreeRTOS.h>
#include <semphr.h>

class PhoneCommunication : public IPhoneCommunication {
  public:
    PhoneCommunication();
    ~PhoneCommunication() = default;

    bool sendBytes(const uint8_t* bytes, uint16_t length) override;
    void registerCallback() override;
    bool isBusy() override;

    void process();

    friend void phoneCommunication_C_txCpltCallback(void* phoneCommunicationInstance);
    friend void phoneCommunication_C_rxCpltCallback(void* phoneCommunicationInstance);

  private:
    enum RxState { waitForHeader, waitForPayload, checkIntegrity };

    bool m_busy;
    uint8_t m_txBuffer[PHONE_COMMUNICATION_MAX_LENGTH];
    uint8_t m_rxBuffer[PHONE_COMMUNICATION_MAX_LENGTH];

    uint8_t m_rxHeader[PHONE_COMMUNICATION_HEADER_LENGTH];
    uint16_t m_rxLength;
    uint32_t m_rxCrc;
    RxState m_rxState;

    SemaphoreHandle_t m_uartSemaphore;

    void txCpltCallback();
    void rxCpltCallback();
};

#endif //__PHONECOMMUNICATION_H__
