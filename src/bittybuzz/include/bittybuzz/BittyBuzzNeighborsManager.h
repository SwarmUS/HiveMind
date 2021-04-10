#ifndef __BITTYBUZZNEIGHBORSMANAGER_H_
#define __BITTYBUZZNEIGHBORSMANAGER_H_

#include "IBittyBuzzNeighborsManager.h"
#include <cpp-common/ICircularQueue.h>
#include <interloc/IInterloc.h>

class BittyBuzzNeighborsManager : public IBittyBuzzNeighborsManager {
  public:
    BittyBuzzNeighborsManager(const IInterloc& interloc, ICircularQueue<uint16_t>& posUpdateQueue);
    ~BittyBuzzNeighborsManager() override = default;

    void updateNeighbors() override;

  private:
    const IInterloc& m_interloc;
    ICircularQueue<uint16_t>& m_posUpdateQueue;
};

#endif // __BITTYBUZZNEIGHBORSMANAGER_H_
