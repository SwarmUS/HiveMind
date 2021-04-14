#ifndef __INTERLOCTIMEMANAGER_H__
#define __INTERLOCTIMEMANAGER_H__

#include <cstdint>

class InterlocTimeManager {
    uint64_t getPollTxTs();
    uint64_t getResponseTimeout();
};

#endif //__INTERLOCTIMEMANAGER_H__
