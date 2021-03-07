#ifndef __BITTYBUZZFUNCTIONDESCRIPTION_H_
#define __BITTYBUZZFUNCTIONDESCRIPTION_H_

#include <array>
#include <hivemind-host/FunctionDescriptionArgumentTypeDTO.h>
#include <hivemind-host/FunctionDescriptionDTO.h>
#include <tuple>

/**
 *@brief Description for a function, used for the FunctionDescriptionRequest/Response */
class BittyBuzzFunctionDescription {
  public:
    /**
     * @b Warning only the pointer is stored so make sure the functionName data lifetime is greater
     * than the funciton description*/
    BittyBuzzFunctionDescription(const char* functionName);

    /**
     *@brief get the array of arguments description
     *@return an reference to an std::array with the information stored*/
    const std::array<std::tuple<const char*, FunctionDescriptionArgumentTypeDTO>,
                     FunctionDescriptionDTO::ARGUMENTS_MAX_SIZE>&
    getArguments() const;

    /**
     *@brief get the length of the array
     *@return the length of teh array*/
    uint16_t getArgumentsLength() const;

    /**
     *@brief add an argument to the list of the description
     *@param argumentName the name of the argument @b Warning, the function description just keeps a
     *pointer to the name, so make sure the data it's points to is valid during usage
     *@param argumentType the type of the argument
     *@return true if the operation was successfull, false if not (no more space in the buffer)*/
    bool addArgument(const char* argumentName, FunctionDescriptionArgumentTypeDTO argumentType);

  private:
    const char* m_functionName;
    std::array<std::tuple<const char*, FunctionDescriptionArgumentTypeDTO>,
               FunctionDescriptionDTO::ARGUMENTS_MAX_SIZE>
        m_argumentDescriptions;
    uint16_t m_argumentDescriptionsLength = 0;
};

#endif // __BITTYBUZZFUNCTIONDESCRIPTION_H_
