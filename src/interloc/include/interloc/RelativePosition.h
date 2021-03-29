#ifndef __RELATIVEPOSITION_H__
#define __RELATIVEPOSITION_H__

#include <cstdint>

struct RelativePosition {
    uint16_t m_robotId;
    float m_distance;
    float m_relativeOrientation;
    bool m_isInLineOfSight;
};

#endif //__RELATIVEPOSITION_H__
