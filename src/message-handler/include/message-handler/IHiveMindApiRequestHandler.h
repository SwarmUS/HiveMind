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
     *@param source the id of the source of the request
     *@param destination the destination of the request (should be the same a the bsp id)
     *@param request the request to handle
     *@return true on success, the request was handled and answered, false if not*/
    virtual bool handleRequest(uint32_t source,
                               uint32_t destination,
                               const HiveMindApiRequestDTO& request) = 0;
};

#endif // __IHIVEMINDAPIREQUESTHANDLER_H_
