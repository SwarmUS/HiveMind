#include "BittyBuzzVmTestsUtils.h"
#include <bbzvm.h>
#include <gtest/gtest.h>

uint8_t g_assertTrueCallCount = 0;
uint8_t g_assertFalseCallCount = 0;

void buzzAssertTrue() {
    g_assertTrueCallCount++;

    bbzvm_assert_lnum(1); // NOLINT
    bbzobj_t* bbzObj = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT
    auto test = bbzfloat_tofloat(bbzObj->f.value);
    (void) test;

    EXPECT_TRUE(bbzObj->i.value);

    bbzvm_ret0();
}

void buzzAssertFalse() {
    g_assertFalseCallCount++;
    bbzvm_assert_lnum(1); // NOLINT
    bbzobj_t* bbzObj = bbzheap_obj_at(bbzvm_locals_at(1)); // NOLINT

    EXPECT_FALSE(bbzObj->i.value);

    bbzvm_ret0();
}
