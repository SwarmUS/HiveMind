#ifndef __IHIVEMINDAPIREQUESTHANDLER_H_
#define __IHIVEMINDAPIREQUESTHANDLER_H_

#include <hivemind-host/HiveMindApiRequestDTO.h>
#include <hivemind-host/MessageDTO.h>

/**
 *@brief Handles HiveMindApi requests and sends the response to the appropriate target */
class IHiveMindApiRequestHandler {
  public:
    virtual ~IHiveMindApiRequestHandler() = default;

    /**
     *@brief handles a request and sends a response to the approriate target
     *@param request the request to handle
     *@return the response to the request */
    virtual HiveMindApiResponseDTO handleRequest(const HiveMindApiRequestDTO& request) = 0;
};

#endif // __IHIVEMINDAPIREQUESTHANDLER_H_
