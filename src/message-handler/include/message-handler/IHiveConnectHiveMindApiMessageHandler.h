#ifndef __IHIVECONNECTHIVEMINDAPIMESSAGEHANDLER_H_
#define __IHIVECONNECTHIVEMINDAPIMESSAGEHANDLER_H_

#include <pheromones/HiveConnectHiveMindApiDTO.h>

/**
 *@brief Handles HiveConnectHiveMindApi messages and makes the proper request
 **/
class IHiveConnectHiveMindApiMessageHandler {
  public:
    virtual ~IHiveConnectHiveMindApiMessageHandler() = default;

    /**
     *@brief handles HiveConnectHiveMindApi
     *@param sourceId the source id of the message
     *@param destId the source id of the message
     *@param message the message dto to handle
     *@return true if the operation was successfull, false if not*/
    virtual bool handleMessage(uint16_t sourceId,
                               uint16_t destId,
                               const HiveConnectHiveMindApiDTO& message) = 0;
};

#endif // __IHIVECONNECTHIVEMINDAPIMESSAGEHANDLER_H_
