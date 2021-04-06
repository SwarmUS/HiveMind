#ifndef __BITTYBUZZNEIGHBORSMANAGER_H_
#define __BITTYBUZZNEIGHBORSMANAGER_H_

#include <cpp-common/ICircularQueue.h>
#include <interloc/IInterloc.h>

class BittyBuzzNeighborsManager {
  public:
    BittyBuzzNeighborsManager(const IInterloc& interloc, ICircularQueue<uint16_t>& posUpdateQueue);
    ~BittyBuzzNeighborsManager();

    bool updateNeighbors();

  private:
    const IInterloc& m_interloc;
    ICircularQueue<uint16_t>& m_posUpdateQueue;
};

#endif // __BITTYBUZZNEIGHBORSMANAGER_H_
