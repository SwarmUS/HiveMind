#include "HiveMindApiRequestHandler.h"

HiveMindApiRequestHandler::HiveMindApiRequestHandler(ICircularQueue<MessageDTO>& hostOutputQ,
                                                     ICircularQueue<MessageDTO>& remoteOutputQ,
                                                     const IBSP& bsp,
                                                     ILogger& logger) :
    m_hostOutputQueue(hostOutputQ),
    m_remoteOutputQueue(remoteOutputQ),
    m_bsp(bsp),
    m_logger(logger) {}

bool HiveMindApiRequestHandler::handleRequest(uint32_t requestId,
                                              uint32_t destination,
                                              const HiveMindApiRequestDTO& request) {

    if (destination != m_bsp.getUUId()) {
        return false;
    }

    const std::variant<std::monostate, IdRequestDTO>& vReq = request.getRequest();
    if (const auto* idReq = std::get_if<IdRequestDTO>(&vReq)) {
        ResponseDTO resp(requestId, IdResponseDTO(m_bsp.getUUId()));
    }

    return false;
}
