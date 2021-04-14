#ifndef __IBITTYBUZZVM_H_
#define __IBITTYBUZZVM_H_

#include "IBittyBuzzLib.h"
#include <bbzvm.h>
#include <functional>

/**
 *@brief Manages BittyBuzz virtual machine */
class IBittyBuzzVm {
  public:
    virtual ~IBittyBuzzVm() = default;

    /**@brief Init the virtual machine
     *@param bbzLibs an array of the libraries to init
     *@param bbzLibsLength the length of the libraries to init*/
    virtual bool init(std::reference_wrapper<IBittyBuzzLib>* bbzLibs, uint32_t bbzLibsLength) = 0;

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
