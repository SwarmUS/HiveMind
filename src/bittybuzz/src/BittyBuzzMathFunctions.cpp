#include "BittyBuzzMathFunctions.h"
#include "BittyBuzzSystem.h"
#include <bbzvm.h>
#include <bsp/Math.h>

void BittyBuzzMathFunctions::bbzmath_abs() {
    bbzvm_assert_lnum(1); // NOLINT

    // Get args
    bbzobj_t* o = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    if (bbztype_isfloat(*o)) {
        bbzvm_pushf(bbzfloat_fromfloat(Math::fabs(bbzfloat_tofloat(o->f.value))));
    } else if (bbztype_isint(*o)) {
        bbzvm_pushi(Math::abs(o->i.value));
    } else {

        bbzvm_seterror(BBZVM_ERROR_MATH);
        return;
    }
    bbzvm_ret1();
}
void BittyBuzzMathFunctions::bbzmath_floor() {
    bbzvm_assert_lnum(1); // NOLINT

    // Get args
    float arg = 0;
    bbzobj_t* o = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    if (bbztype_isfloat(*o)) {
        arg = bbzfloat_tofloat(o->f.value);
    } else if (bbztype_isint(*o)) {
        arg = o->i.value;
    } else {
        bbzvm_seterror(BBZVM_ERROR_MATH);
        return;
    }

    bbzvm_pushi(Math::floor(arg));
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_ceil() {
    bbzvm_assert_lnum(1); // NOLINT

    // Get args
    float arg = 0;
    bbzobj_t* o = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    if (bbztype_isfloat(*o)) {
        arg = bbzfloat_tofloat(o->f.value);
    } else if (bbztype_isint(*o)) {
        arg = o->i.value;
    } else {
        bbzvm_seterror(BBZVM_ERROR_MATH);
        return;
    }

    bbzvm_pushi(Math::ceil(arg));
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_round() {
    bbzvm_assert_lnum(1); // NOLINT

    // Get args
    float arg = 0;
    bbzobj_t* o = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    if (bbztype_isfloat(*o)) {
        arg = bbzfloat_tofloat(o->f.value);
    } else if (bbztype_isint(*o)) {
        arg = o->i.value;
    } else {
        bbzvm_seterror(BBZVM_ERROR_MATH);
        return;
    }

    bbzvm_pushi(Math::ceil(arg));
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_log() {
    bbzvm_assert_lnum(1); // NOLINT

    // Get args
    float arg = 0;
    bbzobj_t* o = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    if (bbztype_isfloat(*o)) {
        arg = bbzfloat_tofloat(o->f.value);
    } else if (bbztype_isint(*o)) {
        arg = o->i.value;
    } else {
        bbzvm_seterror(BBZVM_ERROR_MATH);
        return;
    }

    bbzvm_pushf(bbzfloat_tofloat(Math::ln(arg)));
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_log2() {
    bbzvm_assert_lnum(1); // NOLINT

    float arg = 0;
    bbzobj_t* o = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    if (bbztype_isfloat(*o)) {
        arg = bbzfloat_tofloat(o->f.value);
    } else if (bbztype_isint(*o)) {
        arg = o->i.value;
    } else {
        bbzvm_seterror(BBZVM_ERROR_MATH);
        return;
    }

    bbzvm_pushf(bbzfloat_tofloat(Math::log2(arg)));
    bbzvm_ret1();
}
void BittyBuzzMathFunctions::bbzmath_log10() {
    bbzvm_assert_lnum(1); // NOLINT

    float arg = 0;
    bbzobj_t* o = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    if (bbztype_isfloat(*o)) {
        arg = bbzfloat_tofloat(o->f.value);
    } else if (bbztype_isint(*o)) {
        arg = o->i.value;
    } else {
        bbzvm_seterror(BBZVM_ERROR_MATH);
        return;
    }

    bbzvm_pushf(bbzfloat_tofloat(Math::log10(arg)));
    bbzvm_ret1();
}
void BittyBuzzMathFunctions::bbzmath_exp() {
    bbzvm_assert_lnum(1); // NOLINT

    float arg = 0;
    bbzobj_t* o = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    if (bbztype_isfloat(*o)) {
        arg = bbzfloat_tofloat(o->f.value);
    } else if (bbztype_isint(*o)) {
        arg = o->i.value;
    } else {
        bbzvm_seterror(BBZVM_ERROR_MATH);
        return;
    }

    bbzvm_pushf(bbzfloat_tofloat(Math::exp(arg)));
    bbzvm_ret1();
}
void BittyBuzzMathFunctions::bbzmath_sqrt() {
    bbzvm_assert_lnum(1); // NOLINT

    float arg = 0;
    bbzobj_t* o = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    if (bbztype_isfloat(*o)) {
        arg = bbzfloat_tofloat(o->f.value);
    } else if (bbztype_isint(*o)) {
        arg = o->i.value;
    } else {
        bbzvm_seterror(BBZVM_ERROR_MATH);
        return;
    }

    bbzvm_pushf(bbzfloat_tofloat(Math::sqrt(arg)));
    bbzvm_ret1();
}
void BittyBuzzMathFunctions::bbzmath_sin() {
    bbzvm_assert_lnum(1); // NOLINT

    float arg = 0;
    bbzobj_t* o = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    if (bbztype_isfloat(*o)) {
        arg = bbzfloat_tofloat(o->f.value);
    } else if (bbztype_isint(*o)) {
        arg = o->i.value;
    } else {
        bbzvm_seterror(BBZVM_ERROR_MATH);
        return;
    }

    bbzvm_pushf(bbzfloat_tofloat(Math::sin(arg)));
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_cos() {
    bbzvm_assert_lnum(1); // NOLINT

    float arg = 0;
    bbzobj_t* o = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    if (bbztype_isfloat(*o)) {
        arg = bbzfloat_tofloat(o->f.value);
    } else if (bbztype_isint(*o)) {
        arg = o->i.value;
    } else {
        bbzvm_seterror(BBZVM_ERROR_MATH);
        return;
    }

    bbzvm_pushf(bbzfloat_tofloat(Math::cos(arg)));
    bbzvm_ret1();
}
void BittyBuzzMathFunctions::bbzmath_tan() {
    bbzvm_assert_lnum(1); // NOLINT

    float arg = 0;
    bbzobj_t* o = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    if (bbztype_isfloat(*o)) {
        arg = bbzfloat_tofloat(o->f.value);
    } else if (bbztype_isint(*o)) {
        arg = o->i.value;
    } else {
        bbzvm_seterror(BBZVM_ERROR_MATH);
        return;
    }

    bbzvm_pushf(bbzfloat_tofloat(Math::tan(arg)));
    bbzvm_ret1();
}
void BittyBuzzMathFunctions::bbzmath_asin() {
    bbzvm_assert_lnum(1); // NOLINT

    float arg = 0;
    bbzobj_t* o = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    if (bbztype_isfloat(*o)) {
        arg = bbzfloat_tofloat(o->f.value);
    } else if (bbztype_isint(*o)) {
        arg = o->i.value;
    } else {
        bbzvm_seterror(BBZVM_ERROR_MATH);
        return;
    }

    bbzvm_pushf(bbzfloat_tofloat(Math::asin(arg)));
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_acos() {
    bbzvm_assert_lnum(1); // NOLINT

    float arg = 0;
    bbzobj_t* o = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    if (bbztype_isfloat(*o)) {
        arg = bbzfloat_tofloat(o->f.value);
    } else if (bbztype_isint(*o)) {
        arg = o->i.value;
    } else {
        bbzvm_seterror(BBZVM_ERROR_MATH);
        return;
    }

    bbzvm_pushf(bbzfloat_tofloat(Math::acos(arg)));
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_atan() {
    bbzvm_assert_lnum(2); // NOLINT

    float y = 0;
    bbzobj_t* a = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    if (bbztype_isfloat(*a)) {
        y = bbzfloat_tofloat(a->f.value);
    } else if (bbztype_isint(*a)) {
        y = a->i.value;
    } else {
        bbzvm_seterror(BBZVM_ERROR_MATH);
        return;
    }

    float x = 0;
    bbzobj_t* b = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    if (bbztype_isfloat(*b)) {
        x = bbzfloat_tofloat(b->f.value);
    } else if (bbztype_isint(*b)) {
        x = b->i.value;
    } else {
        bbzvm_seterror(BBZVM_ERROR_MATH);
        return;
    }

    bbzvm_pushf(bbzfloat_tofloat(Math::atan2(y, x)));
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_min() {
    bbzvm_assert_lnum(2); // NOLINT
    /* Get arguments */
    bbzobj_t* a = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    bbzobj_t* b = bbzheap_obj_at(bbzvm_locals_at(2)); // NOLINT

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

    /* Compare them and return the bigger one */
    int cmp = bbztype_cmp(a, b);
    bbzvm_lload(cmp >= 0 ? 1 : 2);
    bbzvm_ret1();
}

void BittyBuzzMathFunctions::bbzmath_rng_uniform() {
    bbzvm_assert_lnum(0); // NOLINT
    uint32_t randomNum = BittyBuzzSystem::g_bsp->generateRandomNumber();
    bbzvm_pushi((int16_t)randomNum);
}
