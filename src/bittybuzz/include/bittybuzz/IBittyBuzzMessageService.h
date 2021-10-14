#ifndef __IBITTYBUZZMESSAGESERVICE_H_
#define __IBITTYBUZZMESSAGESERVICE_H_

#include <pheromones/MessageDTO.h>

/**
 *@brief message service use by the BBVM. Constructs the messages and dispatches it at the
 *appropriate place (queue) */
class IBittyBuzzMessageService {
  public:
    virtual ~IBittyBuzzMessageService() = default;

    /**
     *@brief call a function to a host
     *@param [in] agentId the id of the agent, use 0 for broadcast
     *@param [in] functionName the name of the function to call
     *@param [in] args a list of arguments to pass to the function
     *@param [in] argsLength the number of arguments to pass, the max is
     *FunctionCallRequestDTO::FUNCTION_CALL_ARGUMENTS_MAX_LENGTH*/
    virtual bool callHostFunction(uint16_t agentId,
                                  const char* functionName,
                                  const FunctionCallArgumentDTO* args,
                                  uint16_t argsLength) = 0;

    /**
     *@brief call a function to a buzz
     *@param [in] agentId the id of the agent, use 0 for broadcast
     *@param [in] functionName the name of the function to call
     *@param [in] args a list of arguments to pass to the function
     *@param [in] argsLength the number of arguments to pass, the max is
     *FunctionCallRequestDTO::FUNCTION_CALL_ARGUMENTS_MAX_LENGTH*/
    virtual bool callBuzzFunction(uint16_t agentId,
                                  const char* functionName,
                                  const FunctionCallArgumentDTO* args,
                                  uint16_t argsLength) = 0;
    /**
     *@brief queues the buzz messages
     *@return true if the operation was successfull, false if not (length too big or queue is
     *full)*/
    virtual bool queueBuzzMessages() = 0;
};

#endif // __IBITTYBUZZMESSAGESERVICE_H_
