#ifndef __HOSTUART_H__
#define __HOSTUART_H__

#include "bsp/ICRC.h"
#include "bsp/IHostUart.h"
#include <FreeRTOS.h>
#include <cstdint>
#include <logger/ILogger.h>
#include <semphr.h>

#define HOST_UART_MAX_MESSAGE_LENGTH UINT16_MAX
#define HOST_UART_HEADER_LENGTH 6

class HostUart : public IHostUart {
  public:
    HostUart() = default;
    ~HostUart() override = default;

    bool send(const uint8_t* buffer, uint16_t length) override;
    bool isBusy() override;
};

#endif //__HOSTUART_H__
