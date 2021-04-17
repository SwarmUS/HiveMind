#include "BittyBuzzFactory.h"
#include "BittyBuzzMathFunctions.h"
#include "BittyBuzzUserFunctions.h"
#include <bsp/Math.h>

extern "C" {
#include <main_bytecode.h>
#include <main_string.h>
}

BittyBuzzBytecode BittyBuzzFactory::createBittyBuzzBytecode(ILogger& logger) {
    return BittyBuzzBytecode(logger, bcode, bcode_size);
}

BittyBuzzStringResolver BittyBuzzFactory::createBittyBuzzStringResolver(ILogger& logger) {
    return BittyBuzzStringResolver(g_bbzStringResolverArray.data(), g_bbzStringResolverArray.size(),
                                   BBZSTRING_OFFSET, logger);
}

BittyBuzzLib<std::array<BittyBuzzLibMemberRegister, 10>> BittyBuzzFactory::
    createBittyBuzzGlobalLib() {
    std::array<BittyBuzzLibMemberRegister, 10> globalMember = {{
        {BBZSTRID_log, BittyBuzzUserFunctions::log},
        {BBZSTRID_is_nil, BittyBuzzUserFunctions::isNil},
        {BBZSTRID_is_int, BittyBuzzUserFunctions::isInt},
        {BBZSTRID_is_float, BittyBuzzUserFunctions::isFloat},
        {BBZSTRID_is_string, BittyBuzzUserFunctions::isString},
        {BBZSTRID_is_table, BittyBuzzUserFunctions::isTable},
        {BBZSTRID_is_closure, BittyBuzzUserFunctions::isClosure},
        {BBZSTRID_is_lambda_closure, BittyBuzzUserFunctions::isLambdaClosure},
        {BBZSTRID_register_closure, BittyBuzzUserFunctions::registerClosure},
        {BBZSTRID_call_host_function, BittyBuzzUserFunctions::callHostFunction},
    }};

    return BittyBuzzLib<std::array<BittyBuzzLibMemberRegister, 10>>(globalMember);
}

#include <optional>
std::optional<float> getFloatArg2(uint16_t stackAt) {

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


void bbzmath_mult() {
    bbzvm_assert_lnum(2); // NOLINT

    std::optional<float> y = getFloatArg2(1);
    if (!y) {
        bbzvm_seterror(BBZVM_ERROR_MATH);
        return;
    }

    std::optional<float> x = getFloatArg2(2);
    if (!x) {
        bbzvm_seterror(BBZVM_ERROR_MATH);
        return;
    }

    bbzvm_pushf(bbzfloat_fromfloat(y.value() * x.value()));
    bbzvm_ret1();
}

BittyBuzzLib<std::array<BittyBuzzLibMemberRegister, 21>> BittyBuzzFactory::
    createBittyBuzzMathLib() {
    std::array<BittyBuzzLibMemberRegister, 21> libMember{{
        {BBZSTRID_e, Math::e},
        {BBZSTRID_pi, Math::pi},
        {BBZSTRID_abs, BittyBuzzMathFunctions::bbzmath_abs},
        {BBZSTRID_floor, BittyBuzzMathFunctions::bbzmath_floor},
        {BBZSTRID_ceil, BittyBuzzMathFunctions::bbzmath_ceil},
        {BBZSTRID_round, BittyBuzzMathFunctions::bbzmath_round},
        {BBZSTRID_log, BittyBuzzMathFunctions::bbzmath_log},
        {BBZSTRID_log2, BittyBuzzMathFunctions::bbzmath_log2},
        {BBZSTRID_log10, BittyBuzzMathFunctions::bbzmath_log10},
        {BBZSTRID_exp, BittyBuzzMathFunctions::bbzmath_exp},
        {BBZSTRID_sqrt, BittyBuzzMathFunctions::bbzmath_sqrt},
        {BBZSTRID_sin, BittyBuzzMathFunctions::bbzmath_sin},
        {BBZSTRID_cos, BittyBuzzMathFunctions::bbzmath_cos},
        {BBZSTRID_tan, BittyBuzzMathFunctions::bbzmath_tan},
        {BBZSTRID_asin, BittyBuzzMathFunctions::bbzmath_asin},
        {BBZSTRID_acos, BittyBuzzMathFunctions::bbzmath_acos},
        {BBZSTRID_atan, BittyBuzzMathFunctions::bbzmath_atan},
        {BBZSTRID_min, BittyBuzzMathFunctions::bbzmath_min},
        {BBZSTRID_max, BittyBuzzMathFunctions::bbzmath_max},
        {BBZSTRID_rng_uniform, BittyBuzzMathFunctions::bbzmath_rng_uniform},
        {BBZSTRID_mult, bbzmath_mult}
    }};
    return BittyBuzzLib<std::array<BittyBuzzLibMemberRegister, 21>>(BBZSTRID_math, libMember);
}
