#include "BittyBuzzUIFunctions.h"
#include "BittyBuzzSystem.h"

UserSegment intToUserSegment(int16_t i) {
    if (i < 0) {
        return UserSegment::Zero;
    }
    if (i > 0x0F) {
        return UserSegment::F;
    }
    return static_cast<UserSegment>(i);
}

void BittyBuzzUIFunctions::setLed() {
    bbzvm_assert_lnum(1); // NOLINT
    bbzobj_t* o = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    bool val = static_cast<bool>(bbztype_tobool(o));
    BittyBuzzSystem::g_userUI->setLed(val);
    bbzvm_ret0();
}

void BittyBuzzUIFunctions::setHex() {
    bbzvm_assert_lnum(1); // NOLINT
    bbzobj_t* o = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    if (bbztype_isint(*o)) {
        UserSegment userSegment = intToUserSegment(o->i.value);
        BittyBuzzSystem::g_userUI->setSegment(userSegment);
        bbzvm_ret0();
    } else {
        bbzvm_seterror(BBZVM_ERROR_TYPE);
    }
}
