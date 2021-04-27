#ifndef __INTERLOCUPDATE_H__
#define __INTERLOCUPDATE_H__

#include <cstdint>
#include <optional>

struct InterlocUpdate {
    uint16_t m_robotId{};

    /**
     * @brief Distance in meters
     */
    std::optional<float> m_distance;

    /**
     * @brief Relative orientation in degrees
     */
    std::optional<float> m_relativeOrientation;

    /**
     * @brief Angle of the distant agent in the frame of the current agent (in degrees)
     */
    std::optional<float> m_angleOfArrival;

    std::optional<bool> m_isInLineOfSight;
};

#endif //__INTERLOCUPDATE_H__
