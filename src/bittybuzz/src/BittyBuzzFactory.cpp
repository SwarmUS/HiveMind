#include "BittyBuzzFactory.h"
#include "BittyBuzzMathFunctions.h"
#include "BittyBuzzUIFunctions.h"
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

BittyBuzzLib<std::array<BittyBuzzLibMemberRegister, 14>> BittyBuzzFactory::
    createBittyBuzzGlobalLib() {
    std::array<BittyBuzzLibMemberRegister, 14> globalMember = {{
        {BBZSTRID_log, BittyBuzzUserFunctions::log},
        {BBZSTRID_int, BittyBuzzUserFunctions::toInt},
        {BBZSTRID_float, BittyBuzzUserFunctions::toFloat},
        {BBZSTRID_isnil, BittyBuzzUserFunctions::isNil},
        {BBZSTRID_isint, BittyBuzzUserFunctions::isInt},
        {BBZSTRID_isfloat, BittyBuzzUserFunctions::isFloat},
        {BBZSTRID_isstring, BittyBuzzUserFunctions::isString},
        {BBZSTRID_istable, BittyBuzzUserFunctions::isTable},
        {BBZSTRID_isclosure, BittyBuzzUserFunctions::isClosure},
        {BBZSTRID_islambda_closure, BittyBuzzUserFunctions::isLambdaClosure},
        {BBZSTRID_register_closure, BittyBuzzUserFunctions::registerClosure},
        {BBZSTRID_call_host_function, BittyBuzzUserFunctions::callHostFunction},
        {BBZSTRID_call_buzz_function, BittyBuzzUserFunctions::callBuzzFunction},
        {BBZSTRID_delay, BittyBuzzUserFunctions::delay},
    }};

    return BittyBuzzLib<std::array<BittyBuzzLibMemberRegister, 14>>(globalMember);
}

BittyBuzzLib<std::array<BittyBuzzLibMemberRegister, 24>> BittyBuzzFactory::
    createBittyBuzzMathLib() {
    std::array<BittyBuzzLibMemberRegister, 24> libMember{{
        {BBZSTRID_E, BittyBuzzMathFunctions::E},
        {BBZSTRID_PI, BittyBuzzMathFunctions::PI},
        {BBZSTRID_abs, BittyBuzzMathFunctions::abs},
        {BBZSTRID_floori, BittyBuzzMathFunctions::floori},
        {BBZSTRID_floorf, BittyBuzzMathFunctions::floorf},
        {BBZSTRID_ceili, BittyBuzzMathFunctions::ceili},
        {BBZSTRID_ceilf, BittyBuzzMathFunctions::ceilf},
        {BBZSTRID_roundi, BittyBuzzMathFunctions::roundi},
        {BBZSTRID_roundf, BittyBuzzMathFunctions::roundf},
        {BBZSTRID_log, BittyBuzzMathFunctions::log},
        {BBZSTRID_log2, BittyBuzzMathFunctions::log2},
        {BBZSTRID_log10, BittyBuzzMathFunctions::log10},
        {BBZSTRID_pow, BittyBuzzMathFunctions::pow},
        {BBZSTRID_exp, BittyBuzzMathFunctions::exp},
        {BBZSTRID_sqrt, BittyBuzzMathFunctions::sqrt},
        {BBZSTRID_sin, BittyBuzzMathFunctions::sin},
        {BBZSTRID_cos, BittyBuzzMathFunctions::cos},
        {BBZSTRID_tan, BittyBuzzMathFunctions::tan},
        {BBZSTRID_asin, BittyBuzzMathFunctions::asin},
        {BBZSTRID_acos, BittyBuzzMathFunctions::acos},
        {BBZSTRID_atan, BittyBuzzMathFunctions::atan},
        {BBZSTRID_min, BittyBuzzMathFunctions::min},
        {BBZSTRID_max, BittyBuzzMathFunctions::max},
        {BBZSTRID_rng_uniform, BittyBuzzMathFunctions::rng_uniform},
    }};
    return BittyBuzzLib<std::array<BittyBuzzLibMemberRegister, 24>>(BBZSTRID_math, libMember);
}

BittyBuzzLib<std::array<BittyBuzzLibMemberRegister, 2>> BittyBuzzFactory::createBittyBuzzUILib() {
    std::array<BittyBuzzLibMemberRegister, 2> libMember{
        {{BBZSTRID_set_led, BittyBuzzUIFunctions::setLed},
         {BBZSTRID_set_hex, BittyBuzzUIFunctions::setHex}}};
    return BittyBuzzLib<std::array<BittyBuzzLibMemberRegister, 2>>(BBZSTRID_ui, libMember);
}
