#ifndef __IINTERLOC_H__
#define __IINTERLOC_H__

#include "IInterloc.h"
#include "InterlocSettings.h"
#include "RelativePosition.h"
#include <array>
#include <optional>

struct PositionsTable {
    std::array<RelativePosition, MAX_ROBOTS_IN_SWARM> m_positions;
    uint16_t m_positionsLength;
};

class IInterloc {
  public:
    /**
     * @brief Returns the last known relative position of a given robot
     * @param robotId The ID of the robot to get the position
     * @return The relative position of the robot
     */
    virtual std::optional<RelativePosition> getRobotPosition(uint16_t robotId) const = 0;

    /**
     * @brief Tells if a robot is in line of sight
     * @param robotId The ID of the robot to get
     * @return Is in line of sight
     */
    virtual bool isLineOfSight(uint16_t robotId) const = 0;

    /**
     * @brief Returns the position table for all robots in the swarm
     * @return The position table
     */
    virtual const PositionsTable& getPositionsTable() const = 0;
};

#endif //__IINTERLOC_H__
