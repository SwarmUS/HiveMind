#ifndef __RELATIVEPOSITION_H__
#define __RELATIVEPOSITION_H__

#include <cstdint>

struct RelativePosition {
    uint16_t m_robotId;

    /**
     * @brief Distance in meters
     */
    float m_distance;

    /**
     * @brief Relative orientation in degrees
     */
    float m_relativeOrientation;
    bool m_isInLineOfSight;
};

#endif //__RELATIVEPOSITION_H__
