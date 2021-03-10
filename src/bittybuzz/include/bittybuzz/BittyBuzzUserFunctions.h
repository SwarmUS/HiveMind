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
     *@brief Logs to the default output
     *@details This closure can take a variadic number of arguments
     *@code
     * log_int("Hello world, magic number: ", 42);
     * log_int("Goodbye",  "world", "magic number: ", 42);
     *@endcode */
    void log();

    /**
     *@brief register a new function, exposing it to the remote composant of the swarm
     *@details This closure expects four parameters, one stringId (name of the function), one
     * closure (the function itself), one table with the description of the arguments, and one with
     *the self context
     * The table description of the arguments is an array of tuple, the tuple has the arg name as
     *key and the type, you can use any value, only the type is checked. You could define a variable
     *as int=0 float=0.0 And use those variables for types description
     *@code
     *
     * function registered_function(arg_int, arg_float) {
     *   assert_true(arg_int == 42);
     *    assert_true(arg_float == 42.24);
     * }
     * var args_description = {
     *    .0 = {.arg_int=0},
     *    .1 = {.arg_float=0.0}
     * };
     * register_closure("registeredFunction", registered_function, args_description, nil)
     *@endcode */
    void registerClosure();

    /**
     *@brief calls a function to a host
     *@details This closure expects tree parameters. The is of the host to call (0 for prodcast, id
     *for local host), the name of the function, a table with the list of arguments
     *@code
     * call_host_function(id, "print", {.0 = 42, .1 = 43});
     *@endcode */
    void callHostFunction();

    /**
     *@brief Checks if a variable is nil
     *@details This closure expects one parameter, the variable to verify the type, pushes 1 on
     *true, 0 on false
     *@code
     * if(is_nil(some_val)){
     *   do_stuff();
     * }
     *@endcode */
    void isNil();

    /**
     *@brief Checks if a variable is an int
     *@details This closure expects one parameter, the variable to verify the type, pushes 1 on
     *true, 0 on false
     *@code
     * if(is_int(some_val)){
     *   do_stuff(
     * }
     *@endcode */
    void isInt();

    /**
     *@brief Checks if a variable is a float
     *@details This closure expects one parameter, the variable to verify the type, pushes 1 on
     *true, 0 on false
     *@code
     * if(is_float(some_val)){
     *   do_stuff();
     * }
     *@endcode */
    void isFloat();

    /**
     *@brief Checks if a variable is a float
     *@details This closure expects one parameter, the variable to verify the type, pushes 1 on
     *true, 0 on false
     *@code
     * if(is_string(some_val)){
     *   do_stuff();
     * }
     *@endcode */
    void isString();

    /**
     *@brief Checks if a variable is a table
     *@details This closure expects one parameter, the variable to verify the type, pushes 1 on
     *true, 0 on false
     *@code
     * if(is_table(some_val)){
     *   do_stuff();
     * }
     *@endcode */
    void isTable();

    /**
     *@brief Checks if a variable is a function closure
     *@details This closure expects one parameter, the variable to verify the type, pushes 1 on
     *true, 0 on false.
     *@code
     * if(is_closure(some_val)){
     *   do_stuff();
     * }
     *@endcode */
    void isClosure();

    /**
     *@brief Checks if a variable is a lambda, unamed closure
     *@details This closure expects one parameter, the variable to verify the type, pushes 1 on
     *true, 0 on false
     *@code
     * if(is_lambda_closure(some_val)){
     *   do_stuff();
     * }
     *@endcode */
    void isLambdaClosure();

} // namespace BittyBuzzUserFunctions

#endif // __BITTYBUZZUSERFUNCTIONS_H_
