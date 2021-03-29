#ifndef __INTERLOCUPDATE_H__
#define __INTERLOCUPDATE_H__

#include <cstdint>
#include <optional>

struct InterlocUpdate {
    uint16_t m_robotId{};

    std::optional<float> m_distance;
    std::optional<float> m_relativeOrientation;
    std::optional<bool> m_isInLineOfSight;
};

#endif //__INTERLOCUPDATE_H__
