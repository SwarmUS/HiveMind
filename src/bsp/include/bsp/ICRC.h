#ifndef __ICRC_H__
#define __ICRC_H__

#include <cstdint>

class ICRC {
  public:
    virtual ~ICRC() = default;

    virtual uint32_t calculateCRC32(const void* data, uint32_t length) = 0;
};

#endif //__ICRC_H__
