#ifndef __IBITTYBUZZCLOSUREREGISTER_H_
#define __IBITTYBUZZCLOSUREREGISTER_H_

#include "bittybuzz/BittyBuzzFunctionDescription.h"
#include <bbzvm.h>
#include <optional>

/**@brief a struct containing info of a registered closure*/
struct BittyBuzzRegisteredClosure {
    bbzheap_idx_t m_closureHeapIdx;
    bbzheap_idx_t m_selfHeapIdx;
    BittyBuzzFunctionDescription m_description;
};

/**
 *@brief A class to register buzz function/closures id and associate them with a string. */
class IBittyBuzzClosureRegister {
  public:
    virtual ~IBittyBuzzClosureRegister() = default;

    /**
     *@brief register a closure to the list
     *@param functionName the name of the function
     *@param closureHeapIdx a pointer to the heap to the closure. When registering, the function
     *will be made permanent.
     *@param selfHeapIdx a pointer to the heap to the self variable, can be nil. When registering,
     *the function will be made permanent.
     *@param description the description of the function, it's argument names and types
     *@return true on success, false if not (i.e. no more space in the list, or the heapidx is not a
     *closure)*/
    virtual bool registerClosure(const char* functionName,
                                 bbzheap_idx_t closureHeapIdx,
                                 bbzheap_idx_t selfHeapIdx,
                                 const BittyBuzzFunctionDescription& description) = 0;

    /**
     *@brief get the id of a stored function by it's associated name
     *@param functionName the name of the function to fetch
     *@return an optional that has a reference to the info of the registered closure */
    virtual std::optional<std::reference_wrapper<const BittyBuzzRegisteredClosure>>
    getRegisteredClosure(const char* functionName) const = 0;
};

#endif // __IBITTYBUZZCLOSUREREGISTER_H_
