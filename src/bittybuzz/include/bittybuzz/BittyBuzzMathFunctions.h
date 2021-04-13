#ifndef __BITTYBUZZMATHFUNCTIONS_H_
#define __BITTYBUZZMATHFUNCTIONS_H_

#include "IBittyBuzzMathFunctions.h"
class BittyBuzzMathFunctions : public IBittyBuzzMathFunctions {
    ~BittyBuzzMathFunctions() override = default;

    bool registerMathTable() override;

  private:
    static void bbzmath_abs();
    static void bbzmath_floor();
    static void bbzmath_ceil();
    static void bbzmath_round();
    static void bbzmath_log();
    static void bbzmath_log2();
    static void bbzmath_log10();
    static void bbzmath_exp();
    static void bbzmath_sqrt();
    static void bbzmath_sin();
    static void bbzmath_cos();
    static void bbzmath_tan();
    static void bbzmath_asin();
    static void bbzmath_acos();
    static void bbzmath_atan();
    static void bbzmath_min();
    static void bbzmath_max();
    static void bbzmath_rng_uniform();
};

#endif // __BITTYBUZZMATHFUNCTIONS_H_
