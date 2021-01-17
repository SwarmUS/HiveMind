#ifndef __IPHONECOMMUNICATION_H__
#define __IPHONECOMMUNICATION_H__

#include <cstdint>

class IPhoneCommunication {
  public:
    virtual ~IPhoneCommunication() = default;

    virtual bool sendBytes(const uint8_t* bytes, uint16_t length) = 0;
    virtual void registerCallback() = 0;

    virtual bool isBusy() = 0;
};

#endif //__IPHONECOMMUNICATION_H__
