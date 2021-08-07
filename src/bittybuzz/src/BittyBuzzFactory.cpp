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

BittyBuzzLib<std::array<BittyBuzzLibMemberRegister, 23>> BittyBuzzFactory::
    createBittyBuzzMathLib() {
    std::array<BittyBuzzLibMemberRegister, 23> libMember{{
        {BBZSTRID_e, Math::e},
        {BBZSTRID_pi, Math::pi},
        {BBZSTRID_abs, BittyBuzzMathFunctions::bbzmath_abs},
        {BBZSTRID_floori, BittyBuzzMathFunctions::bbzmath_floori},
        {BBZSTRID_floorf, BittyBuzzMathFunctions::bbzmath_floorf},
        {BBZSTRID_ceili, BittyBuzzMathFunctions::bbzmath_ceili},
        {BBZSTRID_ceilf, BittyBuzzMathFunctions::bbzmath_ceilf},
        {BBZSTRID_roundi, BittyBuzzMathFunctions::bbzmath_roundi},
        {BBZSTRID_roundf, BittyBuzzMathFunctions::bbzmath_roundf},
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
    }};
    return BittyBuzzLib<std::array<BittyBuzzLibMemberRegister, 23>>(BBZSTRID_math, libMember);
}
