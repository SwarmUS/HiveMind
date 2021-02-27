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
     *@details This closure expects one integer parameter, the number to log. Returns nothing */
    void logInt();

    /**
     *@brief Logs a string value
     *@details This closure expects one stringId parameter, the string to log. Returns nothing */
    void logString();

    /**
     *@brief register a new function, exposing it to the remote composant of the swarm
     *@details This closure expects two parameters, one stringId (name of the function), and one
     *closure (the function itself) */
    void registerClosure();

    /**
     *@brief Checks if a variable is nil
     *@details This closure expects one parameter, the variable to verify the type, pushes 1 on
     *true, 0 on false
     * */
    void isNil();

    /**
     *@brief Checks if a variable is an int
     *@details This closure expects one parameter, the variable to verify the type, pushes 1 on
     *true, 0 on false */
    void isInt();

    /**
     *@brief Checks if a variable is a float
     *@details This closure expects one parameter, the variable to verify the type, pushes 1 on
     *true, 0 on false */
    void isFloat();

    /**
     *@brief Checks if a variable is a float
     *@details This closure expects one parameter, the variable to verify the type, pushes 1 on
     *true, 0 on false */
    void isString();

    /**
     *@brief Checks if a variable is a table
     *@details This closure expects one parameter, the variable to verify the type, pushes 1 on
     *true, 0 on false */
    void isTable();

    /**
     *@brief Checks if a variable is a function closure
     *@details This closure expects one parameter, the variable to verify the type, pushes 1 on
     *true, 0 on false */
    void isClosure();

    /**
     *@brief Checks if a variable is a lambda, unamed closure
     *@details This closure expects one parameter, the variable to verify the type, pushes 1 on
     *true, 0 on false */
    void isLambdaClosure();

} // namespace BittyBuzzUserFunctions

#endif // __BITTYBUZZUSERFUNCTIONS_H_
