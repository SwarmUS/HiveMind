#include "bittybuzz/BittyBuzzBytecode.h"

extern "C" {
#include <test_bytecode.h>
}

uint8_t buf[4];
const uint8_t* bbz_bcodeFetcher(bbzpc_t offset, uint8_t size) {
    for (bbzpc_t i = 0; i < size; i++) {
        buf[i] = bcode[i + offset];
    }
    return buf;
}

bbzvm_bcode_fetch_fun BittyBuzzBytecode::getBytecodeFetchFunction() const {
    return bbz_bcodeFetcher;
}

uint16_t BittyBuzzBytecode::getBytecodeLength() const { return bcode_size; }
