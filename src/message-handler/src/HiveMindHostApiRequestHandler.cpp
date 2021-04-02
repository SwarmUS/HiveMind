#include "HiveMindHostApiRequestHandler.h"

HiveMindHostApiRequestHandler::HiveMindHostApiRequestHandler(const IBSP& bsp, ILogger& logger) :
    m_bsp(bsp), m_logger(logger) {}

HiveMindHostApiResponseDTO HiveMindHostApiRequestHandler::handleRequest(
    const HiveMindHostApiRequestDTO& request) {

    (void)request;
    return HiveMindHostApiResponseDTO(GenericResponseDTO(GenericResponseStatusDTO::BadRequest, ""));
}
