#ifndef __BITTYBUZZMATHFUNCTIONS_H_
#define __BITTYBUZZMATHFUNCTIONS_H_

/**@brief Provides math function for the standard lib provided with the vm see IBittyBuzzLib.
 *In the buzz code, the functions are situated in the math namespace */
namespace BittyBuzzMathFunctions {

    /**@brief Calculates the absolute value
     *@b Signature abs(arg1)
     *@details expect one argument
     * -# the value to absolue, an int or a float
     * returns the absolute value
     *@code
     *assert_true(math.abs(-3), 3);
     *@endcode */
    void bbzmath_abs();

    /**@brief Floor a value
     *@b Signature floori(arg1)
     *@details expect one argument
     * -# the value to floor, an int or a float
     * returns the floor of a value as a int
     *@code
     *assert_true(math.floori(3.55), 3);
     *@endcode */
    void bbzmath_floori();

    /**@brief Floor a value
     *@b Signature floorf(arg1)
     *@details expect one argument
     * -# the value to floor, an int or a float
     * returns the floor of a value as a float
     *@code
     *assert_true(math.floorf(3.55), 3.0);
     *@endcode */
    void bbzmath_floorf();

    /**@brief Ceil a value
     *@b Signature ceili(arg1)
     *@details expect one argument
     * -# the value to ceil, an int or a float
     * returns the ceil of a value as int
     *@code
     *assert_true(math.ceili(2.55), 3);
     *@endcode */
    void bbzmath_ceili();

    /**@brief Ceil a value
     *@b Signature ceilf(arg1)
     *@details expect one argument
     * -# the value to ceil, an int or a float
     * returns the ceil of a value as float
     *@code
     *assert_true(math.ceilf(2.55), 3.0);
     *@endcode */
    void bbzmath_ceilf();

    /**@brief Rounds a value
     *@b Signature roundi(arg1)
     *@details expect one argument
     * -# the value to round, an int or a float
     * returns the round of a value as int
     *@code
     *assert_true(math.roundi(2.55), 3);
     *assert_true(math.roundi(2.45), 2);
     *@endcode */
    void bbzmath_roundi();

    /**@brief Rounds a value
     *@b Signature roundf(arg1)
     *@details expect one argument
     * -# the value to round, an int or a float
     * returns the round of a value as float
     *@code
     *assert_true(math.roundf(2.55), 3.0);
     *assert_true(math.roundf(2.45), 2.0);
     *@endcode */
    void bbzmath_roundf();

    /**@brief Calculates the natural logarithm (ln)
     *@b Signature log(arg1)
     *@details expect one argument
     * -# the value whose logaritm is calculated
     * returns the natural logarithm of a value
     *@code
     *var ln_3 = math.log(3);
     *@endcode */
    void bbzmath_log();

    /**@brief Calculates the logarithm base 2
     *@b Signature log2(arg1)
     *@details expect one argument
     * -# the value whose logaritm is calculated
     * returns the logarithm base 2 of a value
     *@code
     *var log2_3 = math.log2(3);
     *@endcode */
    void bbzmath_log2();

    /**@brief Calculates the logarithm base 10
     *@b Signature log10(arg1)
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
     *@b Signature sqrt(arg1)
     *@details expect one argument
     * -# the value to square root
     * returns the value of the square root
     *@code
     *var sqrt_3 = math.sqrt(3);
     *@endcode */
    void bbzmath_sqrt();

    /**@brief Calculates the sine
     *@b Signature sin(arg1)
     *@details expect one argument
     * -# the value to calculate the sine, in degrees
     *   returns the sine of the value
     *@code
     *var sin_3 = math.sin(3);
     *@endcode */
    void bbzmath_sin();

    /**@brief Calculates the cosine
     *@b Signature cos(arg1)
     *@details expect one argument
     * -# the value to calculate the cosine, in degrees
     * returns the cosine of the value
     *@code
     *var cos_3 = math.cos(3);
     *@endcode */
    void bbzmath_cos();

    /**@brief Calculates the tengent
     *@b Signature tan(arg1)
     *@details expect one argument
     * -# the value to calculate the tengent, in degrees
     * returns the tengent of the value
     *@code
     *var tan_3 = math.tan(3);
     *@endcode */
    void bbzmath_tan();

    /**@brief Calculates the arc sine
     *@b Signature asin(arg1)
     *@details expect one argument
     * -# the value to calculate the arc sine, the value must be betwen [-1, +1]
     * returns the arc sine of the value in degrees
     *@code
     *var asin_03 = math.asin(0.3);
     *@endcode */
    void bbzmath_asin();

    /**@brief Calculates the arc cosine
     *@b Signature acos(arg1)
     *@details expect one argument
     * -# the value to calculate the arc cosine, the value must be betwen [-1, +1]
     * returns the arc cosine of the value in degrees
     *@code
     *var acos_03 = math.acos(0.3);
     *@endcode */
    void bbzmath_acos();

    /**@brief Calculates the arc cosine
     *@b Signature atan(arg1)
     *@details expect two arguments
     * -# the value representing the y coordinate
     * -# the value representing the x coordinate
     * returns the arc tengent of y/x in degrees
     *@code
     *var atan_03 = math.atan(0.3);
     *@endcode */
    void bbzmath_atan();

    /**@brief Return the min
     *@b Signature min(arg1, arg2)
     *@details expect two arguments
     * -# the first value
     * -# the second value
     * returns the minimum of the two arguments
     *@code
     *assert_true(math.min(2,3), 2);
     *@endcode */
    void bbzmath_min();

    /**@brief Return the max
     *@b Signature max(arg1, arg2)
     *@details expect two arguments
     * -# the first value
     * -# the second value
     * returns the max of the two arguments
     *@code
     *assert_true(math.min(2,3), 3);
     *@endcode */

    void bbzmath_max();

    /**@brief Return a number with a uniform distribution
     *@b Signature rng_uniform()
     *@details expects no arguments
     * returns a random number, follows a uniform distribution
     *@code
     * var random_number = math.rng_uniform();
     *@endcode */
    void bbzmath_rng_uniform();

} // namespace BittyBuzzMathFunctions

#endif // __BITTYBUZZMATHFUNCTIONS_H_
