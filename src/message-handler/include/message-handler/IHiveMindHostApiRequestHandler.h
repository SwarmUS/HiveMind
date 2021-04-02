#ifndef __IHIVEMINDHOSTAPIREQUESTHANDLER_H_
#define __IHIVEMINDHOSTAPIREQUESTHANDLER_H_

#include <optional>
#include <pheromones/HiveMindHostApiRequestDTO.h>
#include <pheromones/MessageDTO.h>

/**
 *@brief Handles HiveMindApi requests and sends the response to the appropriate target */
class IHiveMindHostApiRequestHandler {
  public:
    virtual ~IHiveMindHostApiRequestHandler() = default;

    /**
     *@brief handles HiveMindHostAPI request and sends a response to the approriate target
     *@param message the message dto to handle
     *@return true if the operation was successfull, false if not*/
    virtual bool handleRequest(const MessageDTO& message) = 0;
};

#endif // __IHIVEMINDHOSTAPIREQUESTHANDLER_H_
