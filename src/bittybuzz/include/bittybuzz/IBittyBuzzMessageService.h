#ifndef __IBITTYBUZZMESSAGESERVICE_H_
#define __IBITTYBUZZMESSAGESERVICE_H_

#include <hivemind-host/MessageDTO.h>

/**
 *@brief message service use by the BBVM. Constructs the messages and dispatches it at the
 *appropriate place (queue) */
class IBittyBuzzMessageService {
  public:
    virtual ~IBittyBuzzMessageService() = default;

    /**
     *@brief call a function to a host
     *@param [in] hostId the id of the host, use 0 for broadcast
     *@param [in] functionName the name of the function to call
     *@param [in] args a list of arguments to pass to the function
     *@param [in] argsLength the number of arguments to pass, the max is
     *FunctionCallRequestDTO::FUNCTION_CALL_ARGUMENTS_MAX_LENGTH*/
    virtual bool callHostFunction(uint16_t hostId,
                                  const char* functionName,
                                  const FunctionCallArgumentDTO* args,
                                  uint16_t argsLength) = 0;
    /**
     *@brief send a buzz message, broadcasted to other buzz vm, sends it to the remote queue
     *@param [in] msg the buzz message to send
     *@return true if the operation was successfull, false if not (length too big or queue is
     *full)*/
    virtual bool sendBuzzMessage(const BuzzMessageDTO& msg) = 0;
};

#endif // __IBITTYBUZZMESSAGESERVICE_H_
