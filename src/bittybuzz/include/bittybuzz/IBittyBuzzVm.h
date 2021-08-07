#ifndef __IBITTYBUZZVM_H_
#define __IBITTYBUZZVM_H_

#include "IBittyBuzzLib.h"
#include <bbzvm.h>
#include <functional>

/*@brief Return codes of the VM when executing*/
enum class BBVMRet { Ok = 0, VmErr = 1, OutMsgErr = 2 };

/**
 *@brief Manages BittyBuzz virtual machine */
class IBittyBuzzVm {
  public:
    virtual ~IBittyBuzzVm() = default;

    /**@brief Init the virtual machine
     *@param bbzLibs an array of the libraries to init
     *@param bbzLibsLength the length of the libraries to init*/
    virtual bool init(const std::reference_wrapper<IBittyBuzzLib>* bbzLibs,
                      uint32_t bbzLibsLength) = 0;

    /** @brief Does one execution step in the virtual machine.  Thus execute the buzz code in the
     * step function */
    virtual BBVMRet step() = 0;

    /** @brief Get the state of the vm */
    virtual bbzvm_state getState() const = 0;

    /** @brief Get the error state of the VM */
    virtual bbzvm_error getError() const = 0;

    /** @brief Get the current instruction of the VM */
    virtual bbzvm_instr getInstruction() const = 0;
};

#endif // __IBITTYBUZZVM_H_
