#include "BittyBuzzStdLib.h"
#include "BittyBuzzMathFunctions.h"
#include <bbzvm.h>

void BittyBuzzStdLib::registerLibs() { registerMathLib(); }

void BittyBuzzStdLib::registerMathLib() {
    bbzvm_pusht(); // "math"

    bbztable_add_function(BBZSTRING_ID(min), bbz_min);
    bbztable_add_function(BBZSTRING_ID(min), bbz_min);

    bbzvm_gsym_register(BBZSTRING_ID(math), bbzvm_stack_at(0)); // Register sybole
    bbzvm_pop();
}
