#ifndef __BITTYBUZZMATHFUNCTIONS_H_
#define __BITTYBUZZMATHFUNCTIONS_H_

#include "IBittyBuzzMathFunctions.h"
class BittyBuzzMathFunctions : public IBittyBuzzMathFunctions {
    ~BittyBuzzMathFunctions() override = default;

    bool registerMathTable() override;

  private:
    void bbzmath_abs();
    void bbzmath_floor();
    void bbzmath_ceil();
    void bbzmath_round();
    void bbzmath_log();
    void bbzmath_log2();
    void bbzmath_log10();
    void bbzmath_exp();
    void bbzmath_sqrt();
    void bbzmath_sin();
    void bbzmath_cos();
    void bbzmath_tan();
    void bbzmath_asin();
    void bbzmath_acos();
    void bbzmath_atan();
    void bbzmath_min();
    void bbzmath_max();
    void bbzmath_rng();
};

#endif // __BITTYBUZZMATHFUNCTIONS_H_
