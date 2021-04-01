#ifndef __IHIVEMINDHOSTAPIREQUESTHANDLER_H_
#define __IHIVEMINDHOSTAPIREQUESTHANDLER_H_

#include <pheromones/HiveMindHostApiRequestDTO.h>
#include <pheromones/MessageDTO.h>

/**
 *@brief Handles HiveMindApi requests and sends the response to the appropriate target */
class IHiveMindHostApiRequestHandler {
  public:
    virtual ~IHiveMindHostApiRequestHandler() = default;

    /**
     *@brief handles a request and sends a response to the approriate target
     *@param request the request to handle
     *@return the response to the request */
    virtual HiveMindHostApiResponseDTO handleRequest(const HiveMindHostApiRequestDTO& request) = 0;
};

#endif // __IHIVEMINDHOSTAPIREQUESTHANDLER_H_
