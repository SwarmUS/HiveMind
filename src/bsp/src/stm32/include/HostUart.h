#ifndef __HOSTUART_H__
#define __HOSTUART_H__

#include "bsp/ICRC.h"
#include "bsp/IHostUart.h"
#include <FreeRTOS.h>
#include <cstdint>
#include <logger/ILogger.h>
#include <semphr.h>

#define HOST_UART_MAX_MESSAGE_LENGTH UINT16_MAX
#define HOS_UART_HEADER_LENGTH 6

class HostUart : public IHostUart {
  public:
    explicit HostUart(ICRC& crc, ILogger& logger);
    ~HostUart() override = default;

    bool sendBytes(const uint8_t* bytes, uint16_t length) override;
    void registerCallback() override;
    bool isBusy() override;

    void process();

    friend void phoneCommunication_C_txCpltCallback(void* phoneCommunicationInstance);
    friend void phoneCommunication_C_rxCpltCallback(void* phoneCommunicationInstance);

  private:
    enum RxState { waitForHeader, waitForPayload, checkIntegrity };

    ICRC& m_crc;
    ILogger& m_logger;

    bool m_busy;
    uint8_t m_txBuffer[HOST_UART_MAX_MESSAGE_LENGTH];
    uint8_t m_rxBuffer[HOST_UART_MAX_MESSAGE_LENGTH];

    uint8_t m_rxHeader[HOS_UART_HEADER_LENGTH];
    uint16_t m_rxLength;
    uint32_t m_rxCrc;
    RxState m_rxState;

    SemaphoreHandle_t m_uartSemaphore;

    void txCpltCallback();
    void rxCpltCallback();
};

#endif //__HOSTUART_H__
