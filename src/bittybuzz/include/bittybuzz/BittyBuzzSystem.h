#ifndef __BITTYBUZZSYSTEM_H_
#define __BITTYBUZZSYSTEM_H_

#include "IBittyBuzzFunctionRegister.h"
#include "IBittyBuzzStringResolver.h"
#include <bbzvm.h>
#include <logger/ILogger.h>

// TODO: Change to a class once bittybuzz support passing context to function and make it a
// dependency for the BBVM

/**
 *@brief Namespace to help manage the bittybuzz system.
 **/
namespace BittyBuzzSystem {

    /**
     *@brief Logger used for error reception or user functions
     **/
    extern ILogger* g_logger;

    /**
     *@brief String resolver used for user functions
     **/
    extern const IBittyBuzzStringResolver* g_stringResolver;

    /**
     *@brief Function register used by the BBVM to register new functions
     **/
    extern IBittyBuzzFunctionRegister* g_functionRegister;

    /**
     *@brief Call a bittybuzz function that takes not arguments (init, step, etc),
     *
     *@param [in] stringId the string ID of the function
     */
    void functionCall(uint16_t stringId);

    /**
     *@brief Callback to handle errors on bittybuzz vm
     *
     *@param [in] errcode the error code of the received error */
    void errorReceiver(bbzvm_error errcode);

} // namespace BittyBuzzSystem

#endif // __BITTYBUZZSYSTEM_H_
