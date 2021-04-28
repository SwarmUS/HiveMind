#include "HiveMindHostApiRequestHandler.h"

HiveMindHostApiRequestHandler::HiveMindHostApiRequestHandler(
    const IBSP& bsp,
    ICircularQueue<MessageDTO>& hostQueue,
    ICircularQueue<MessageDTO>& remoteQueue,
    const IInterloc& interloc,
    ILogger& logger) :
    m_bsp(bsp),
    m_hostQueue(hostQueue),
    m_remoteQueue(remoteQueue),
    m_interloc(interloc),
    m_logger(logger) {}

bool HiveMindHostApiRequestHandler::handleRequest(const MessageDTO& message) {

    if (const auto* req = std::get_if<RequestDTO>(&message.getMessage())) {
        if (const auto* hivemindHostApiReq =
                std::get_if<HiveMindHostApiRequestDTO>(&req->getRequest())) {

            return handleHiveMindHostApiRequest(req->getId(), message, *hivemindHostApiReq);
        }
    }

    m_logger.log(LogLevel::Warn, "Received Unknown message in api request handler");
    return false;
}

bool HiveMindHostApiRequestHandler::handleHiveMindHostApiRequest(
    uint16_t requestId, const MessageDTO& message, const HiveMindHostApiRequestDTO& request) {

    if (std::holds_alternative<BytesDTO>(request.getRequest())) {
        return m_hostQueue.push(message);
    }

    if (const auto* neighborReq = std::get_if<GetNeighborRequestDTO>(&request.getRequest())) {
        uint16_t id = neighborReq->getNeighborId();
        std::optional<RelativePosition> posOpt = m_interloc.getRobotPosition(id);

        std::optional<NeighborPositionDTO> posOptDTO = {};
        if (posOpt) {

            const RelativePosition& pos = posOpt.value();
            posOptDTO = NeighborPositionDTO(pos.m_distance, pos.m_relativeOrientation,
                                            pos.m_isInLineOfSight);
        }

        GetNeighborResponseDTO neighborResp(id, posOptDTO);
        HiveMindHostApiResponseDTO hmResp(neighborResp);
        ResponseDTO resp(requestId, hmResp);
        MessageDTO msg(m_bsp.getUUId(), message.getSourceId(), resp);
        return m_hostQueue.push(msg);
    }

    if (std::holds_alternative<GetNeighborsListRequestDTO>(request.getRequest())) {

        GetNeighborsListResponseDTO neighborResp(NULL, 0);
        const PositionsTable& posTable = m_interloc.getPositionsTable();

        neighborResp.setRawNeighborsLength(posTable.m_positionsLength);
        auto& rawNeighbors = neighborResp.getRawNeighbors();
        for (uint16_t i = 0; i < neighborResp.getNeighborsLength(); i++) {
            rawNeighbors[i] = posTable.m_positions[i].m_robotId;
        }

        HiveMindHostApiResponseDTO hmResp(neighborResp);
        ResponseDTO resp(requestId, hmResp);
        MessageDTO msg(m_bsp.getUUId(), message.getSourceId(), resp);
        return m_hostQueue.push(msg);
    }

    if (const auto* req = std::get_if<GetAgentsListRequestDTO>(&request.getRequest())) {
        // TODO: What happens if comes from a remote?
        MessageDTO msg(message.getSourceId(), message.getDestinationId(),
                       HiveConnectHiveMindApiDTO(requestId, *req));
        return m_remoteQueue.push(message);
    }

    m_logger.log(LogLevel::Warn, "Received unknown hivemind host api");
    return false;
}
