#ifndef __BITTYBUZZSYSTEM_H_
#define __BITTYBUZZSYSTEM_H_

#include <bbzvm.h>
#include <logger/ILogger.h>

// Since the bittybuzz callbacks does not support context, we use a namespace instead of an class
// object, could have been a static class or static function in class.

/**
 *@brief Namespace to help manage the bittybuzz system.
 **/
namespace bbz_system {

    /**
     *@brief Logger used for error reception or user functions
     **/
    extern const ILogger* logger;

    /**
     *@brief Call a bittybuzz function, check the function documentation and verify if you need to
     *pass parameter via the vm stack
     *
     *@param [in] strid the string ID of the function*/
    void functionCall(uint16_t strid);

    /**
     *@brief Callback to handle errors on bittybuzz vm
     *
     *@param [in] errcode the error code of the received error */
    void errorReceiver(bbzvm_error errcode);

} // namespace bbz_system

#endif // __BITTYBUZZSYSTEM_H_
