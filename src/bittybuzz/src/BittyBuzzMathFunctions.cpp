#include "BittyBuzzMathFunctions.h"
#include "BittyBuzzSystem.h"
#include <bbzvm.h>
#include <bsp/Math.h>
#include <optional>

bool isNumber(bbzobj_t* obj) { return bbztype_isint(*obj) || bbztype_isfloat(*obj); }

std::optional<float> getFloatArg(uint16_t stackAt) {

    float arg = 0;
    bbzobj_t* o = bbzheap_obj_at(bbzvm_locals_at(stackAt)); // NOLINT
    if (bbztype_isfloat(*o)) {
        arg = bbzfloat_tofloat(o->f.value);
    } else if (bbztype_isint(*o)) {
        arg = o->i.value;
    } else {
        return {};
    }
    return arg;
}

typedef void (*callbackFloatArg)(float arg);
void callIfNumber(uint16_t argPos, callbackFloatArg callback) {
    std::optional<float> arg = getFloatArg(argPos);
    if (!arg) {
        bbzvm_seterror(BBZVM_ERROR_MATH);
        return;
    }
    callback(arg.value());
}

void BittyBuzzMathFunctions::bbzmath_abs() {
    bbzvm_assert_lnum(1); // NOLINT

    // Get args
    bbzobj_t* o = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    if (bbztype_isfloat(*o)) {
        bbzvm_pushf(bbzfloat_fromfloat(Math::fabs(bbzfloat_tofloat(o->f.value))));
    } else if (bbztype_isint(*o)) {
        bbzvm_pushi(Math::abs(o->i.value));
    } else {
        bbzvm_seterror(BBZVM_ERROR_TYPE);
        return;
    }
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_floori() {
    bbzvm_assert_lnum(1); // NOLINT
    auto fun = [](float arg) { bbzvm_pushi(Math::floor(arg)); };
    callIfNumber(1, fun);
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_floorf() {
    bbzvm_assert_lnum(1); // NOLINT
    auto fun = [](float arg) { bbzvm_pushf(bbzfloat_fromfloat(Math::floor(arg))); };
    callIfNumber(1, fun);
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_ceili() {
    bbzvm_assert_lnum(1); // NOLINT
    auto fun = [](float arg) { bbzvm_pushi(Math::ceil(arg)); };
    callIfNumber(1, fun);
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_ceilf() {
    bbzvm_assert_lnum(1); // NOLINT
    auto fun = [](float arg) { bbzvm_pushf(bbzfloat_fromfloat(Math::ceil(arg))); };
    callIfNumber(1, fun);
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_roundi() {
    bbzvm_assert_lnum(1); // NOLINT
    auto fun = [](float arg) { bbzvm_pushi(Math::ceil(arg)); };
    callIfNumber(1, fun);
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_roundf() {
    bbzvm_assert_lnum(1); // NOLINT
    auto fun = [](float arg) { bbzvm_pushf(bbzfloat_fromfloat(Math::ceil(arg))); };
    callIfNumber(1, fun);
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_log() {
    bbzvm_assert_lnum(1); // NOLINT

    // Get args
    std::optional<float> arg = getFloatArg(1);
    if (!arg) {
        bbzvm_seterror(BBZVM_ERROR_TYPE);
        return;
    }

    bbzvm_pushf(bbzfloat_fromfloat(Math::ln(arg.value())));
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_log2() {
    bbzvm_assert_lnum(1); // NOLINT

    std::optional<float> arg = getFloatArg(1);
    if (!arg) {
        bbzvm_seterror(BBZVM_ERROR_TYPE);
        return;
    }

    bbzvm_pushf(bbzfloat_fromfloat(Math::log2(arg.value())));
    bbzvm_ret1();
}
void BittyBuzzMathFunctions::bbzmath_log10() {
    bbzvm_assert_lnum(1); // NOLINT

    std::optional<float> arg = getFloatArg(1);
    if (!arg) {
        bbzvm_seterror(BBZVM_ERROR_TYPE);
        return;
    }

    bbzvm_pushf(bbzfloat_fromfloat(Math::log10(arg.value())));
    bbzvm_ret1();
}
void BittyBuzzMathFunctions::bbzmath_exp() {
    bbzvm_assert_lnum(1); // NOLINT

    std::optional<float> arg = getFloatArg(1);
    if (!arg) {
        bbzvm_seterror(BBZVM_ERROR_TYPE);
        return;
    }

    bbzvm_pushf(bbzfloat_fromfloat(Math::exp(arg.value())));
    bbzvm_ret1();
}
void BittyBuzzMathFunctions::bbzmath_sqrt() {
    bbzvm_assert_lnum(1); // NOLINT

    std::optional<float> arg = getFloatArg(1);
    if (!arg) {
        bbzvm_seterror(BBZVM_ERROR_TYPE);
        return;
    }

    bbzvm_pushf(bbzfloat_fromfloat(Math::sqrt(arg.value())));
    bbzvm_ret1();
}
void BittyBuzzMathFunctions::bbzmath_sin() {
    bbzvm_assert_lnum(1); // NOLINT

    std::optional<float> arg = getFloatArg(1);
    if (!arg) {
        bbzvm_seterror(BBZVM_ERROR_TYPE);
        return;
    }

    bbzvm_pushf(bbzfloat_fromfloat(Math::sin(arg.value() * Math::pi / 180)));
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_cos() {
    bbzvm_assert_lnum(1); // NOLINT

    std::optional<float> arg = getFloatArg(1);
    if (!arg) {
        bbzvm_seterror(BBZVM_ERROR_TYPE);
        return;
    }

    bbzvm_pushf(bbzfloat_fromfloat(Math::cos(arg.value() * Math::pi / 180)));
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_tan() {
    bbzvm_assert_lnum(1); // NOLINT

    std::optional<float> arg = getFloatArg(1);
    if (!arg) {
        bbzvm_seterror(BBZVM_ERROR_TYPE);
        return;
    }

    bbzvm_pushf(bbzfloat_fromfloat(Math::tan(arg.value() * Math::pi / 180)));
    bbzvm_ret1();
}
void BittyBuzzMathFunctions::bbzmath_asin() {
    bbzvm_assert_lnum(1); // NOLINT

    std::optional<float> arg = getFloatArg(1);
    if (!arg) {
        bbzvm_seterror(BBZVM_ERROR_TYPE);
        return;
    }

    bbzvm_pushf(bbzfloat_fromfloat(Math::asin(arg.value()) * 180 / Math::pi));
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_acos() {
    bbzvm_assert_lnum(1); // NOLINT

    std::optional<float> arg = getFloatArg(1);
    if (!arg) {
        bbzvm_seterror(BBZVM_ERROR_TYPE);
        return;
    }

    bbzvm_pushf(bbzfloat_fromfloat(Math::acos(arg.value()) * 180 / Math::pi));
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_atan() {
    bbzvm_assert_lnum(2); // NOLINT

    std::optional<float> y = getFloatArg(1);
    if (!y) {
        bbzvm_seterror(BBZVM_ERROR_TYPE);
        return;
    }

    std::optional<float> x = getFloatArg(2);
    if (!x) {
        bbzvm_seterror(BBZVM_ERROR_TYPE);
        return;
    }

    bbzvm_pushf(bbzfloat_fromfloat(Math::atan2(y.value(), x.value()) * 180 / Math::pi));
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_min() {
    bbzvm_assert_lnum(2); // NOLINT
    /* Get arguments */
    bbzobj_t* a = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    bbzobj_t* b = bbzheap_obj_at(bbzvm_locals_at(2)); // NOLINT
    if (!isNumber(a) || !isNumber(b)) {
        bbzvm_seterror(BBZVM_ERROR_TYPE);
        return;
    }

    /* Compare them and return the smaller one */
    int cmp = bbztype_cmp(a, b);
    bbzvm_lload(cmp >= 0 ? 1 : 2);
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_max() {
    bbzvm_assert_lnum(2); // NOLINT
    /* Get arguments */
    bbzobj_t* a = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    bbzobj_t* b = bbzheap_obj_at(bbzvm_locals_at(2)); // NOLINT
    if (!isNumber(a) || !isNumber(b)) {
        bbzvm_seterror(BBZVM_ERROR_TYPE);
        return;
    }

    /* Compare them and return the bigger one */
    int cmp = bbztype_cmp(a, b);
    bbzvm_lload(cmp >= 0 ? 1 : 2);
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_rng_uniform() {
    bbzvm_assert_lnum(0); // NOLINT
    int16_t randomNum = static_cast<int16_t>(BittyBuzzSystem::g_bsp->generateRandomNumber());
    bbzvm_pushi(randomNum);
    bbzvm_ret1();
}
