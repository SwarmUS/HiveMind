#ifndef __HOSTUART_H__
#define __HOSTUART_H__

#include "bsp/IHostUart.h"

class HostUart : public IHostUart {
  public:
    HostUart() = default;
    ~HostUart() override = default;

    bool send(const uint8_t* buffer, uint16_t length) override;
    bool isBusy() override;
};

#endif //__HOSTUART_H__
