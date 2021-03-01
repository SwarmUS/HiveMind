#ifndef __IBITTYBUZZMESSAGESERVICE_H_
#define __IBITTYBUZZMESSAGESERVICE_H_

#include <hivemind-host/MessageDTO.h>

class IBittyBuzzMessageService {
  public:
    virtual ~IBittyBuzzMessageService() = default;

    virtual bool callFunction(uint16_t id,
                              const char* functionName,
                              const FunctionCallArgumentDTO* args,
                              uint16_t argsLength) = 0;
};

#endif // __IBITTYBUZZMESSAGESERVICE_H_
