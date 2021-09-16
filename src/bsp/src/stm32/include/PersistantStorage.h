#ifndef __PERSITEDSTORAGE_H__
#define __PERSITEDSTORAGE_H__

#include <cstdint>
// This struct should be world alligned on instantiation. Use __attribute__ ((aligned (4))).
struct PersistedStorage {
    // The size should be a multiple of words for flash operations
    static uint16_t getSize() {
        return sizeof(PersistedStorage) + 4 - sizeof(PersistedStorage) % 4;
    }
    uint16_t m_uuid;
};

#endif //__PERSITEDSTORAGE_H__
