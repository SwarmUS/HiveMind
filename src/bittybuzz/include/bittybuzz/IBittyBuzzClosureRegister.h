#ifndef __IBITTYBUZZCLOSUREREGISTER_H_
#define __IBITTYBUZZCLOSUREREGISTER_H_

#include <bbzvm.h>
#include <optional>

/**
 *@brief A class to register buzz function/closures id and associate them with a string. */
class IBittyBuzzClosureRegister {
  public:
    virtual ~IBittyBuzzClosureRegister() = default;

    /**
     *@brief register un function to the list
     *@param functionName the name of the function
     *@param closureHeapIdx a pointer to the heap to the closure. When registering, the function
     *will be made permanent.
     *@return true on success, false if not (i.e. no more space in the list, or the heapidx is not a
     *closure)*/
    virtual bool registerClosure(const char* functionName, bbzheap_idx_t closureHeapIdx) = 0;

    /**
     *@brief get the id of a stored function by it's associated name
     *@param functionName the name of the function to fetch
     *@return an optional that has the ID of the function if it's in the list. Nothing if not*/
    virtual std::optional<bbzheap_idx_t> getClosureHeapIdx(const char* functionName) const = 0;
};

#endif // __IBITTYBUZZCLOSUREREGISTER_H_
