#include "HiveMindHostApiRequestHandler.h"

HiveMindHostApiRequestHandler::HiveMindHostApiRequestHandler(const IBSP& bsp, ILogger& logger) :
    m_bsp(bsp), m_logger(logger) {}

HiveMindHostApiResponseDTO HiveMindHostApiRequestHandler::handleRequest(
    const HiveMindHostApiRequestDTO& request) {

    const std::variant<std::monostate, IdRequestDTO>& vReq = request.getRequest();
    if (std::holds_alternative<IdRequestDTO>(vReq)) {
        return HiveMindHostApiResponseDTO(IdResponseDTO(m_bsp.getUUId()));
    }

    return HiveMindHostApiResponseDTO(GenericResponseDTO(GenericResponseStatusDTO::BadRequest, ""));
}
