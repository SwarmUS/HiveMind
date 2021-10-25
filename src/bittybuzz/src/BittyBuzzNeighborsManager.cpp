#include "BittyBuzzNeighborsManager.h"
#include "bbzvm.h"
#include <cstdio>

BittyBuzzNeighborsManager::BittyBuzzNeighborsManager(uint16_t id,
                                                     const IInterloc& interloc,
                                                     ICircularQueue<uint16_t>& posUpdateQueue) :
    m_id(id), m_interloc(interloc), m_posUpdateQueue(posUpdateQueue) {}

void BittyBuzzNeighborsManager::updateNeighbors() {

    // uint32_t length = m_posUpdateQueue.getLength();
    PositionsTable posTable = m_interloc.getPositionsTable();
    for (uint32_t i = 0; i<posTable.m_positionsLength; i++){
        const RelativePosition&  pos = posTable.m_positions[i];
        bbzneighbors_elem_t neighbor;
        neighbor.robot = pos.m_robotId;
        neighbor.azimuth = bbzfloat_fromfloat(pos.m_angle);
        neighbor.distance = bbzfloat_fromfloat(pos.m_distance);
        bbzneighbors_add(&neighbor);
    }
    /**
    for (uint32_t i = 0; i < length; i++) {
        auto robotId = m_posUpdateQueue.peek();
        if (robotId) {
            m_posUpdateQueue.pop();
            std::optional<RelativePosition> posOpt = m_interloc.getRobotPosition(robotId.value());
            if (posOpt) {
                printf("%d: UPDATING ID %d \n", m_id, robotId.value().get());
                RelativePosition& pos = posOpt.value();
                bbzneighbors_elem_t neighbor;
                neighbor.robot = robotId.value();
                neighbor.azimuth = bbzfloat_fromfloat(pos.m_angle);
                neighbor.distance = bbzfloat_fromfloat(pos.m_distance);
                bbzneighbors_add(&neighbor);
            }
        }
    }**/
}
