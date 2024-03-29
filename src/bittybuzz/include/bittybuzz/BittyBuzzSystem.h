#ifndef __BITTYBUZZSYSTEM_H_
#define __BITTYBUZZSYSTEM_H_

#include "IBittyBuzzClosureRegister.h"
#include "IBittyBuzzMessageService.h"
#include "IBittyBuzzStringResolver.h"
#include <application-interface/IUserUI.h>
#include <bbzvm.h>
#include <bsp/IBSP.h>
#include <bsp/IUserInterface.h>
#include <logger/ILogger.h>

// TODO: Change to a class once bittybuzz support passing context to function and make it a
// dependency for the BBVM

/**
 *@brief Namespace to help manage the bittybuzz system.
 **/
namespace BittyBuzzSystem {

    /**
     *@brief UserInterface used by the bbvm for printing user logs
     **/
    extern IUserInterface* g_ui;

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
    extern IBittyBuzzClosureRegister* g_closureRegister;

    /**
     *@brief Message service used by the BBVM to send requests to host and remote
     **/
    extern IBittyBuzzMessageService* g_messageService;

    /**
     *@brief BSP used by the bbvm for random numbers
     **/
    extern IBSP* g_bsp;

    /**
     *@brief User interface (LED, Hex display, etc) available to the user via buzz
     **/
    extern IUserUI* g_userUI;

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

    /**
     *@brief Logs the vm dump which contains information about the vm state, heap, stack, etc.
     *
     *@param [in] logLevel the log level */
    void logVmDump(LogLevel logLevel);

    const char* getStateString(bbzvm_state state);

    const char* getErrorString(bbzvm_error error);

    const char* getInstructionString(bbzvm_instr instruction);

} // namespace BittyBuzzSystem

#endif // __BITTYBUZZSYSTEM_H_
