#include "BittyBuzzNeighborsManager.h"
#include "bbzvm.h"

BittyBuzzNeighborsManager::BittyBuzzNeighborsManager(const IInterloc& interloc,
                                                     ICircularQueue<uint16_t>& posUpdateQueue) :
    m_interloc(interloc), m_posUpdateQueue(posUpdateQueue) {}

void BittyBuzzNeighborsManager::updateNeighbors() {

    uint32_t length = m_posUpdateQueue.getLength();
    for (uint32_t i = 0; i < length; i++) {
        auto robotId = m_posUpdateQueue.peek();
        if (robotId) {
            m_posUpdateQueue.pop();
            std::optional<RelativePosition> posOpt = m_interloc.getRobotPosition(robotId.value());

            if (posOpt) {
                // TODO fix position with buzz, cast from float to uint8_t overflow, 360 degrees
                // does not fit in 8 bits
                RelativePosition& pos = posOpt.value();
                bbzneighbors_elem_t neighbor;
                neighbor.robot = robotId.value();
                neighbor.azimuth = (uint8_t)pos.m_relativeOrientation;
                neighbor.distance = (uint8_t)pos.m_distance;
                neighbor.elevation = 0;
                bbzneighbors_add(&neighbor);
            }
        }
    }
}