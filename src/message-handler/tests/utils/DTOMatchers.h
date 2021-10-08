#ifndef __DTOMATCHERS_H_
#define __DTOMATCHERS_H_

#include <gmock/gmock.h>
#include <pheromones/MessageDTO.h>

// Simple matcher to check that DTO matches inside function call. If required
MATCHER_P(MessageGreetingDTOMatcher, expectedId, "Equality matcher for NetworkApiDTO)") {
    auto greet = std::get<GreetingDTO>(arg.getMessage());
    return greet.getId() == expectedId;
}

MATCHER_P(MessageGetNeighborResponseDTOMatcher,
          pos,
          "Equality matcher for GetNeighborResponseDTO)") {
    auto resp = std::get<ResponseDTO>(arg.getMessage());
    auto hive = std::get<HiveMindHostApiResponseDTO>(resp.getResponse());
    auto getNeighbor = std::get<GetNeighborResponseDTO>(hive.getResponse());

    bool match = getNeighbor.getNeighborId() == pos.m_robotId;
    auto posOpt = getNeighbor.getNeighborPosition();
    match = match && pos.m_distance == posOpt.value().getDistance();
    match = match && pos.m_angle == posOpt.value().getAzimuth();
    match = match && pos.m_isInLineOfSight == posOpt.value().inLOS();
    return match;
}

MATCHER_P(MessageGetEmtpyNeighborResponseDTOMatcher,
          ignored,
          "Equality matcher for empty pos GetNeighborResponseDTO)") {
    auto resp = std::get<ResponseDTO>(arg.getMessage());
    auto hive = std::get<HiveMindHostApiResponseDTO>(resp.getResponse());
    auto getNeighbor = std::get<GetNeighborResponseDTO>(hive.getResponse());

    auto posOpt = getNeighbor.getNeighborPosition();
    return !posOpt.has_value();
}

MATCHER_P(MessageGetNeighborsListResponseDTOMatcher,
          vectRobotId,
          "Equality matcher for GetNeighborsListResponseDTO)") {
    auto resp = std::get<ResponseDTO>(arg.getMessage());
    auto hive = std::get<HiveMindHostApiResponseDTO>(resp.getResponse());
    auto getNeighbor = std::get<GetNeighborsListResponseDTO>(hive.getResponse());

    auto neighbors = getNeighbor.getNeighbors();
    auto neighborsLenght = getNeighbor.getNeighborsLength();
    bool match = neighborsLenght == vectRobotId.size();
    for (uint16_t i = 0; i < neighborsLenght; i++) {
        match = match && neighbors[i] == vectRobotId[i];
    }

    return match;
}

#endif // __DTOMATCHERS_H_
