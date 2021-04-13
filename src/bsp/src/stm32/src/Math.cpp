#include "bsp/Math.h"
#include <math.h>

// TODO: Use cmsis math library

float Math::cos(float x) { return ::cos(x); }

float Math::sin(float x) { return ::sin(x); }

float Math::tan(float x) { return ::tan(x); }

float Math::acos(float x) { return ::acos(x); }

float Math::asin(float x) { return ::asin(x); }

float Math::atan(float x) { return ::atan(x); }

float Math::atan2(float y, float x) { return ::atan2(y, x); }

float Math::pow(float base, float exponent) { return ::pow(base, exponent); }

float Math::sqrt(float x) { return ::sqrt(x); }

float Math::cbrt(float x) { return ::cbrt(x); }

float Math::exp(float x) { return ::exp(x); }

float Math::ln(float x) { return ::log(x); }

float Math::log10(float x) { return ::log10(x); }

float Math::log2(float x) { return ::log2(x); }

float Math::ceil(float x) { return ::ceil(x); }

float Math::floor(float x) { return ::floor(x); }

float Math::fmod(float numer, float denom) { return ::fmod(numer, denom); }

float Math::round(float x) { return ::round(x); }

float Math::fdim(float x, float y) { return ::fdim(x, y); }

float Math::fmax(float x, float y) { return ::fmax(x, y); }

float Math::fmin(float x, float y) { return ::fmin(x, y); }

float Math::fabs(float x) { return ::fabs(x); }

int Math::abs(int x) { return ::abs(x); }
