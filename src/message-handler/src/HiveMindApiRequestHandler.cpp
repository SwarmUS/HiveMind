#include "HiveMindApiRequestHandler.h"

HiveMindApiRequestHandler::HiveMindApiRequestHandler(ICircularQueue<MessageDTO>& hostOutputQ,
                                                     ICircularQueue<MessageDTO>& remoteOutputQ,
                                                     const IBSP& bsp,
                                                     ILogger& logger) :
    m_hostOutputQueue(hostOutputQ),
    m_remoteOutputQueue(remoteOutputQ),
    m_bsp(bsp),
    m_logger(logger) {}

HiveMindApiResponseDTO HiveMindApiRequestHandler::handleRequest(
    const HiveMindApiRequestDTO& request) {

    const std::variant<std::monostate, IdRequestDTO>& vReq = request.getRequest();
    if (std::holds_alternative<IdRequestDTO>(vReq)) {
        return HiveMindApiResponseDTO(IdResponseDTO(m_bsp.getUUId()));
    }

    return HiveMindApiResponseDTO(GenericResponseDTO(GenericResponseStatusDTO::BadRequest, ""));
}
