#ifndef __BITTYBUZZUSERFUNCTIONS_H_
#define __BITTYBUZZUSERFUNCTIONS_H_

// TODO: Change to a class once bittybuzz support passing context to function, and inject string
// resolver here instead of the BittyBuzzSystem

/**
 *@brief Namespace to regroup the user functions, ie: custom functions that will be available in the
 *buzz script. The functions needs to be registered using bbzvm_function_register
 **/
namespace BittyBuzzUserFunctions {

    /**
     *@brief Logs a number value. Use it for sanity checks, it has no real production values
     *
     *@details This closure expects one integer parameter, the number to log. Returns nothing
     * */
    void logInt();

    /**
     *@brief Logs a string value
     *
     *@details This closure expects one stringId parameter, the string to log. Returns nothing
     * */
    void logString();

} // namespace BittyBuzzUserFunctions

#endif // __BITTYBUZZUSERFUNCTIONS_H_
