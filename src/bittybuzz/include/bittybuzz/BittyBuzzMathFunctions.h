#ifndef __BITTYBUZZMATHFUNCTIONS_H_
#define __BITTYBUZZMATHFUNCTIONS_H_

/**@brief Provides math function for the standard lib provided with the vm see IBittyBuzzLib */
namespace BittyBuzzMathFunctions {

    /**@brief Calculates the absolute value
     *@details expect one argument
     * -# the value to absolue, an int or a float
     * returns the absolute value
     *@code
     *assert_true(math.abs(-3), 3);
     *@endcode */
    void bbzmath_abs();

    /**@brief Floor a value
     *@details expect one argument
     * -# the value to floor, an int or a float
     * returns the floor of a value
     *@code
     *assert_true(math.floor(3.55), 3);
     *@endcode */
    void bbzmath_floor();

    /**@brief Ceil a value
     *@details expect one argument
     * -# the value to ceil, an int or a float
     * returns the ceil of a value
     *@code
     *assert_true(math.ceil(2.55), 3);
     *@endcode */
    void bbzmath_ceil();

    /**@brief Rounds a value
     *@details expect one argument
     * -# the value to round, an int or a float
     * returns the round of a value
     *@code
     *assert_true(math.round(2.55), 3);
     *assert_true(math.round(2.45), 2);
     *@endcode */
    void bbzmath_round();

    /**@brief Calculates the natural logarithm (ln)
     *@details expect one argument
     * -# the value whose logaritm is calculated
     * returns the natural logarithm of a value
     *@code
     *var ln_3 = math.log(3);
     *@endcode */
    void bbzmath_log();

    /**@brief Calculates the logarithm base 2
     *@details expect one argument
     * -# the value whose logaritm is calculated
     * returns the logarithm base 2 of a value
     *@code
     *var log2_3 = math.log2(3);
     *@endcode */
    void bbzmath_log2();

    /**@brief Calculates the logarithm base 10
     *@details expect one argument
     * -# the value whose logaritm is calculated
     * returns the logarithm base 10 of a value
     *@code
     *var log10_3 = math.log10(3);
     *@endcode */
    void bbzmath_log10();

    /**@brief Calculates the exponent
     *@details expect one argument
     * -# the value of the exponent
     * returns the value of the exponent
     *@code
     *var exp_3 = math.exp(3);
     *@endcode */
    void bbzmath_exp();

    /**@brief Calculates the square root
     *@details expect one argument
     * -# the value to square root
     * returns the value of the square root
     *@code
     *var sqrt_3 = math.sqrt(3);
     *@endcode */
    void bbzmath_sqrt();

    /**@brief Calculates the sine
     *@details expect one argument
     * -# the value to calculate the sine, in degrees
     * returns the sine of the value*/
    void bbzmath_sin();

    /**@brief Calculates the cosine
     *@details expect one argument
     * -# the value to calculate the cosine, in degrees
     * returns the cosine of the value*/
    void bbzmath_cos();

    /**@brief Calculates the tengent
     *@details expect one argument
     * -# the value to calculate the tengent, in degrees
     * returns the tengent of the value*/
    void bbzmath_tan();

    /**@brief Calculates the arc sine
     *@details expect one argument
     * -# the value to calculate the arc sine, the value must be betwen [-1, +1]
     * returns the arc sine of the value in degrees*/
    void bbzmath_asin();

    /**@brief Calculates the arc cosine
     *@details expect one argument
     * -# the value to calculate the arc cosine, the value must be betwen [-1, +1]
     * returns the arc cosine of the value in degrees*/
    void bbzmath_acos();

    /**@brief Calculates the arc cosine
     *@details expect two arguments
     * -# the value representing the y coordinate
     * -# the value representing the x coordinate
     * returns the arc tengent of y/x in degrees*/
    void bbzmath_atan();

    /**@brief Return the min
     *@details expect two arguments
     * -# the first value
     * -# the second value
     * returns the minimum of the two arguments
     *@code
     *assert_true(math.min(2,3), 2);
     *@endcode */

    void bbzmath_min();

    /**@brief Return the max
     *@details expect two arguments
     * -# the first value
     * -# the second value
     * returns the max of the two arguments
     *@code
     *assert_true(math.min(2,3), 3);
     *@endcode */

    void bbzmath_max();

    /**@brief Return a number with a uniform distribution
     *@details expects no arguments
     * returns a random number, follows a uniform distribution
     *@code
     * var random_number = math.rng_uniform();
     *@endcode */
    void bbzmath_rng_uniform();

} // namespace BittyBuzzMathFunctions

#endif // __BITTYBUZZMATHFUNCTIONS_H_
