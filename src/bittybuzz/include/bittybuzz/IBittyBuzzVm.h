#ifndef __IBITTYBUZZVM_H_
#define __IBITTYBUZZVM_H_

#include "BittyBuzzUserFunctionRegister.h"
#include "IBittyBuzzLib.h"
#include <bbzvm.h>

/**
 *@brief Manages BittyBuzz virtual machine */
class IBittyBuzzVm {
  public:
    virtual ~IBittyBuzzVm() = default;

    /**@brief Init the virtual machine
     *@param functions an array of functions to register
     *@param functionsLength the length of the functions array
     *@param bbzLibs the libraries to register on init*/
    virtual bool init(const BittyBuzzUserFunctionRegister* functions,
                      uint32_t functionsLength,
                      IBittyBuzzLib& bbzLibs) = 0;

    /** @brief Does one execution step in the virtual machine.  Thus execute the buzz code in the
     * step function
     * @return true if the operation was successful, false if not. */
    virtual bool step() = 0;

    /** @brief Get the state of the vm */
    virtual bbzvm_state getSate() const = 0;

    /** @brief Get the error state of the VM */
    virtual bbzvm_error getError() const = 0;
};

#endif // __IBITTYBUZZVM_H_
