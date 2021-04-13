#ifndef __MATH_H_
#define __MATH_H_

/**
 *@brief Namespace that provides basic mathematical operations. Optized for the platform*/
namespace Math {

    /**@brief calculate the cosine
     *@param x value in radian
     *@return the cosine of x radians*/
    float cos(float x);

    /**@brief calculate the sine
     *@param x value in radian
     *@return the sine of x radians*/
    float sin(float x);

    /**@brief calculate the tangent
     *@param x value in radian
     *@return the tangent of x radians*/
    float tan(float x);

    /**@brief calculate the arc cosine
     *@param x value to compute, interval of [-1, +1]. An error occur if out of the interval
     *@return the arc cosine of x in radians [0, pi]*/
    float acos(float x);

    /**@brief calculate the arc sine
     *@param x value to compute, interval of [-1, +1]. An error occur if out of the interval
     *@return the arc sine of x in radians [0, pi]*/
    float asin(float x);

    /**@brief calculate the arc tangent
     *@param x value to compute.
     *@return the arc tangent of x in radians [-pi/2, pi/2]*/
    float atan(float x);

    /**@brief calculate the arc of x/y
     *@param y value representing the y coordinate
     *@param x value representing the x coordinate
     *@return the arc tangent of y/x in radians [-pi, pi]*/
    float atan2(float y, float x);

    /**@brief Calculates the base raised to the power of the exponent
     *@param base the base value
     *@param exponent the exponent value
     *@return the base raised to the power of the exponent*/
    float pow(float base, float exponent);

    /**@brief Calculates the square root
     *@param x the value to square root. If x is negative, an error occure
     *@return the square root of x*/
    float sqrt(float x);

    /**@brief Calculates the cube root
     *@param x the value to cube root. If x is negative, an error occure
     *@return the cube root of x*/
    float cbrt(float x);

    /**@brief Calculates the exponent
     *@param x the value of the exponent
     *@return the value of the exponent*/
    float exp(float x);

    /**@brief Calculates the natural logarithm
     *@param x the value whose logarithm is calculated
     *@return the natural logarithm of x*/
    float ln(float x);

    /**@brief Calculates the logarithm base 10
     *@param x the value whose logarithm is calculated
     *@return the logarithm base 10 of x*/
    float log10(float x);

    /**@brief Calculates the logarithm base 2
     *@param x the value whose logarithm is calculated
     *@return the logarithm base 2 of x*/
    float log2(float x);

    /**@brief Round x upward
     *@param x the value to round up
     *@return the smallest integral value that is not less than x (as float)*/
    float ceil(float x);

    /**@brief Floor x downward
     *@param x the value to floor down
     *@return the smallest integral value that is not greater than x (as float)*/
    float floor(float x);

    /**@brief Calculates the remainder of numer/denom
     *@param numer the quotient numerator
     *@param denom the quotient denominator of 0, an error occurs
     *@return the remainder of divinding the argument */
    float fmod(float numer, float denom);

    /**@brief Round x
     *@param x the value to round
     *@return the value of x rounded to the nearest integral (as float)*/
    float round(float x);

    /**@brief Returns the positive difference between x and y
     *@param x the value to be subtracted
     *@param y the value to subtract
     *@return x-y if x>y. zero otherwise*/
    float fdim(float x, float y);

    /**@brief Return the larger of it's arguments, either x or y
     *@param x one of the value to select the maximum
     *@param y one of the value to select the maximum
     *@return the maximum value of it's arguments*/
    float fmax(float x, float y);

    /**@brief Return the minimum of it's arguments, either x or y
     *@param x one of the value to select the minimum
     *@param y one of the value to select the minimum
     *@return the minimum value of it's arguments*/
    float fmin(float x, float y);

    /**@brief Return the absolute value
     *@param x value whose absolute value is returned
     *@return the absolute value value */
    float fabs(float x);

    /**@brief Return the absolute value
     *@param x value whose absolute value is returned
     *@return the absolute value value */
    int abs(int x);

} // namespace Math

#endif // __MATH_H_
