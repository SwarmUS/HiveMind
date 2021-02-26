#ifndef __IBITTYBUZZFUNCTIONREGISTER_H_
#define __IBITTYBUZZFUNCTIONREGISTER_H_

#include <bbzvm.h>
#include <optional>

/**
 *@brief A class to register buzz function id and associate them with a string. */
class IBittyBuzzFunctionRegister {
  public:
    virtual ~IBittyBuzzFunctionRegister() = default;

    /**
     *@brief register un function to the list
     *@param functionName the name of the function
     *@param functionHeapIdx a pointer to the heap to the function. When registering, the function
     *will be made permanent.
     *@return true on success, false if not (i.e. no more space in the list, or the heapidx is not a
     *closure)*/
    virtual bool registerFunction(const char* functionName, bbzheap_idx_t functionHeapIdx) = 0;

    /**
     *@brief get the id of a stored function by it's associated name
     *@param functionName the name of the function to fetch
     *@return an optional that has the ID of the function if it's in the list. Nothing if not*/
    virtual std::optional<bbzheap_idx_t> getFunctionHeapIdx(const char* functionName) const = 0;
};

#endif // __IBITTYBUZZFUNCTIONREGISTER_H_
