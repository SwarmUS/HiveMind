#ifndef __PERSITEDSTORAGE_H__
#define __PERSITEDSTORAGE_H__

#include <cstdint>
#include <interloc/AngleCalculator.h>

// This struct should be world alligned on instantiation. Use __attribute__ ((aligned (4))).
struct PersistedStorage {
    uint16_t m_uuid;
    AngleCalculatorParameters m_angleCalculatorParameters;
};

#endif //__PERSITEDSTORAGE_H__
