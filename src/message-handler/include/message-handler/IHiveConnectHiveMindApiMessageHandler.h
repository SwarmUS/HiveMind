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
     *@param message the message dto to handle
     *@return true if the operation was successfull, false if not*/
    virtual bool handleMessage(const HiveConnectHiveMindApiDTO& apiMessage) = 0;
};

#endif // __IHIVECONNECTHIVEMINDAPIMESSAGEHANDLER_H_
