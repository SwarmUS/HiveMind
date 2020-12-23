#include "bittybuzz/bittybuzz.h"
#include <bbzvm.h>
#include <bsp/bsp_info.h>

BittyBuzz::BittyBuzz() {}
void BittyBuzz::start_vm() { bbzvm_construct(BSPInfo::get_id()); }
