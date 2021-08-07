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
     *@b Signature log(args...)
     *@details This closure can take variadic arguments. So the number and type can vary.
     *@code
     * log("Hello world, magic number: ", 42);
     * log("Goodbye",  "world", "magic number: ", 42);
     *@endcode */
    void log();

    /**
     *@brief register a new function, exposing it to the remote composant of the swarm
     *@b Signature register_closure(fname, closure, args_desc)
     *@details This closure expects four parameters, the closure can be a lambda.
     * -# stringId (name of the function)
     * -# a closure (the callback function)
     * -# a table with the description of the arguments
     * -# one with the self context
     *
     * The table description of the arguments is an array of tuples, which hold the arg name as key
     *and a value (float or int), only the type is check, so the value is not considered.You could
     *define a variable as int=0 float=0.0 and use those variables for types description
     *@code
     *
     * var context = {.some_context = 42};
     *
     * function registered_function(arg_int, arg_float) {
     *   log("Context: ", self.some_context, "Arg int: ", arg_int, "Arg float: ", arg_float);
     * }
     * var args_description = {
     *    .0 = {.arg_int=0}, // We just pass value so the type is registered
     *    .1 = {.arg_float=0.0} // Same here
     * };
     * register_closure("registeredFunction", registered_function, args_description, context);
     *@endcode */
    void registerClosure();

    /**
     *@brief calls a function to a host
     *@b Signature call_host_function(host_id, fname, params)
     *@details This closure expects three parameters.
     * -# the id of the host to call (0 for broadcast, id for local host)
     * -# the name of the function
     * -# a table with the list of arguments
     *
     * The table argument's must be by index from 0 to N-1 args.
     *
     *@code
     * call_host_function(id, "print", {.0 = 42, .1 = 43});
     *@endcode */
    void callHostFunction();

    /**
     *@brief Checks if a variable is nil
     *@b Signature is_nil(arg1)
     *@details This closure expects one parameter
     * -# the variable to verify the type
     * returns 1 on true, 0 on false
     *@code
     * if(is_nil(some_val)){
     *   do_stuff();
     * }
     *@endcode */
    void isNil();

    /**
     *@brief Checks if a variable is an int
     *@b Signature is_int(arg1)
     *@details This closure expects one parameter
     * -# the variable to verify the type
     * returns 1 on true, 0 on false
     *@code
     * if(is_int(some_val)){
     *   do_stuff(
     * }
     *@endcode */
    void isInt();

    /**
     *@brief Checks if a variable is a float
     *@details This closure expects one parameter
     * -# the variable to verify the type
     * returns 1 on true, 0 on false
     *@code
     * if(is_float(some_val)){
     *   do_stuff();
     * }
     *@endcode */
    void isFloat();

    /**
     *@brief Checks if a variable is a float
     *@b Signature is_table(arg1)
     *@details This closure expects one parameter
     * -# the variable to verify the type
     * returns 1 on true, 0 on false
     *@code
     * if(is_string(some_val)){
     *   do_stuff();
     * }
     *@endcode */
    void isString();

    /**
     *@brief Checks if a variable is a table
     *@b Signature is_table(arg1)
     *@details This closure expects one parameter
     * -# the variable to verify the type
     * returns 1 on true, 0 on false
     *@code
     * if(is_table(some_val)){
     *   do_stuff();
     * }
     *@endcode */
    void isTable();

    /**
     *@brief Checks if a variable is a function closure
     *@b Signature is_closure(arg1)
     *@details This closure expects one parameter
     * -# the variable to verify the type
     * returns 1 on true, 0 on false
     *@code
     * if(is_closure(some_val)){
     *   do_stuff();
     * }
     *@endcode */
    void isClosure();

    /**
     *@brief Checks if a variable is a lambda, unamed closure
     *@b Signature is_lambda_closure(arg1)
     *@details This closure expects one parameter
     * -# the variable to verify the type
     * returns 1 on true, 0 on false
     *@code
     * if(is_lambda_closure(some_val)){
     *   do_stuff();
     * }
     *@endcode */
    void isLambdaClosure();

    /**
     *@brief Casts a type to int
     *@b Signature int(arg1)
     *@details This closure expects one parameter
     * -# the variable to cast
     * returns the casted value of the variable
     * Only works with float, if the type is not supported, nil is returned
     *@code
     * let f = 3.14;
     * let i = int(f);
     * function_that_takes_int(i);
     *@endcode */
    void toInt();

    /**
     *@brief Casts a type to float
     *@b Signature float(arg1)
     *@details This closure expects one parameter
     * -# the variable to cast
     * returns the casted value of the variable
     * Only works with int, if the type is not supported, nil is returned
     *@code
     * let i = 3;
     * let f = float(3);
     * function_that_takes_float(f);
     * @endcode */
    void toFloat();

} // namespace BittyBuzzUserFunctions

#endif // __BITTYBUZZUSERFUNCTIONS_H_
