#ifndef __ICRC_H__
#define __ICRC_H__

#include <cstdint>

class ICRC {
  public:
    virtual ~ICRC() = default;

    /**
     * @brief Calculates the CRC32 of a buffer (Thread-safe)
     * @param data Pointer to the buffer
     * @param length Length of buffer in bytes
     * @return CRC32
     */
    virtual uint32_t calculateCRC32(const void* data, uint32_t length) = 0;
};

#endif //__ICRC_H__
