#ifndef __BITTYBUZZUSERFUNCTIONS_H_
#define __BITTYBUZZUSERFUNCTIONS_H_

// Since the bittybuzz callbacks does not support context, we use a namespace instead of an class
// object, could have been a static class or static function in class.
//
/**
 *@brief Namespace to regroup the user functions, ie: custom functions that will be available in the
 *buzz script. The functions needs to be registered using bbzvm_function_register
 **/
namespace bbz_user_functions {

    /**
     *@brief Logs a number value. Use it for sanity checks, it has no real production values
     *
     * @details This closure expects one parameter: the number to log
     * */
    void logInt();

} // namespace bbz_user_functions

#endif // __BITTYBUZZUSERFUNCTIONS_H_
