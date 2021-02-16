#ifndef __BITTYBUZZVMTESTSUTILS_H_
#define __BITTYBUZZVMTESTSUTILS_H_

#include <cstdint>

extern uint8_t g_assertTrueCallCount;
extern uint8_t g_assertFalseCallCount;

void buzzAssertTrue();

void buzzAssertFalse();

#endif // __BITTYBUZZVMTESTSUTILS_H_
