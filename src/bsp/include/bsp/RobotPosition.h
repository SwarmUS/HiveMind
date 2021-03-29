#ifndef __ROBOTPOSITION_H__
#define __ROBOTPOSITION_H__

#include <cstdint>

struct RobotPosition {
    uint16_t m_robotId;
    float m_distance;
    float m_relativeOrientation;
    bool m_isInLineOfSight;
};

#endif //__ROBOTPOSITION_H__
