//
// Created by hubert on 3/4/21.
//

#ifndef HIVE_MIND_IUSB_H
#define HIVE_MIND_IUSB_H

#include <common/IProtobufStream.h>

class IUSB : public IProtobufStream {
  public:
    virtual ~IUSB() = default;
    virtual bool send(const uint8_t* buffer, uint16_t length) = 0;
    virtual bool receive(uint8_t* buffer, uint16_t length) = 0;
};
#endif // HIVE_MIND_IUSB_H
