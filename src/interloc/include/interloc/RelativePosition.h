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
     * @brief Angle of other agent in current agent frame (in degrees)
     */
    float m_angle;

    float m_angleRealMean;
    float m_angleImaginaryMean;

    bool m_isInLineOfSight;
};

#endif //__RELATIVEPOSITION_H__
