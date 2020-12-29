#include "bittybuzz/BittyBuzzBytecode.h"
#include <functional>


uint8_t buf[4];
const uint8_t* bbz_bcodeFetcher(bbzpc_t offset, uint8_t size) {
    for (bbzpc_t i = 0; i < size; i++) {
        buf[i] = bcode[i + offset];
    }
    return buf;
}

BittyBuzzBytecode::BittyBuzzBytecode(const uint8_t* bytecode = bcode, const uint16_t bytecodeSize = bcode_size) {
    m_bytecode = bcode;
    m_bytecodeSize = bytecodeSize;

}

bbzvm_bcode_fetch_fun BittyBuzzBytecode::getBytecodeFetchFunction() const {

    const uint8_t* (*test)(bbzpc_t offset, uint8_t size) =  [] (bbzpc_t offset, uint8_t size){
        return (const uint8_t*) 1;
    };


    std::function<const uint8_t*(bbzpc_t, uint8_t)> test2([this] (bbzpc_t offset, uint8_t size){
        return (const uint8_t*) 1;
    });

    test = test2;
}

uint16_t BittyBuzzBytecode::getBytecodeLength() const { return m_bytecodeSize; }
