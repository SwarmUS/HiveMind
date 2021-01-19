#ifndef __IHOSTUART_H__
#define __IHOSTUART_H__

#include <cstdint>

class IHostUart {
  public:
    virtual ~IHostUart() = default;

    virtual bool sendBytes(const uint8_t* bytes, uint16_t length) = 0;
    virtual void registerCallback() = 0;

    virtual bool isBusy() = 0;
};

#endif //__IHOSTUART_H__
