#ifndef __IBITTYBUZZFUNCTIONREGISTER_H_
#define __IBITTYBUZZFUNCTIONREGISTER_H_

#include <optional>

/**
 *@brief A class to register buzz function id and associate them with a string. */
class IBittyBuzzFunctionRegister {
  public:
    virtual ~IBittyBuzzFunctionRegister() = default;

    /**
     *@brief register un function to the list
     *@param functionName the name of the function
     *@param functionId the string id of the function in the buzz code
     *@return true on success, false if not (i.e. no more space in the list)*/
    virtual bool registerFunction(const char* functionName, uint16_t functionId) = 0;

    /**
     *@brief get the id of a stored function by it's associated name
     *@param functionName the name of the function to fetch
     *@return an optional that has the ID of the function if it's in the list. Nothing if not*/
    virtual std::optional<uint16_t> getFunctionId(const char* functionName) const = 0;
};

#endif // __IBITTYBUZZFUNCTIONREGISTER_H_
