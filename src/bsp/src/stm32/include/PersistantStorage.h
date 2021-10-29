#ifndef __PERSITEDSTORAGE_H__
#define __PERSITEDSTORAGE_H__

#include <cstdint>
#include <interloc/AngleCalculator.h>

// This struct should be world alligned on instantiation. Use __attribute__ ((aligned (4))).
struct __attribute__((packed, aligned(4))) PersistedStorage {
    uint16_t m_uuid;

    __attribute__((packed, aligned(4)))
    AngleCalculatorParameters m_angleCalculatorParameters;
};

#endif //__PERSITEDSTORAGE_H__
