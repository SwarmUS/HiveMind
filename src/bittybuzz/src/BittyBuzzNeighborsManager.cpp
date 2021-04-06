#include "BittyBuzzNeighborsManager.h"

BittyBuzzNeighborsManager::BittyBuzzNeighborsManager(const IInterloc& interloc,
                                                     ICircularQueue<uint16_t>& posUpdateQueue) :
    m_interloc(interloc), m_posUpdateQueue(posUpdateQueue) {}

BittyBuzzNeighborsManager::~BittyBuzzNeighborsManager() {}

bool BittyBuzzNeighborsManager::updateNeighbors() {
    uint32_t length = m_posUpdateQueue.getLength();
    for (uint32_t i = 0; i < length; i++) {
        m_posUpdateQueue.peek();
    }
    return false;
}
