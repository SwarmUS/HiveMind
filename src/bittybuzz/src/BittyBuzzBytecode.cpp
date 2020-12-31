#include "bittybuzz/BittyBuzzBytecode.h"
#include "logger/ILogger.h"
#include <functional>

// TODO: Make a PR to bittybuzz to pass context
static const ILogger* g_logger = NULL;
static const uint8_t* g_bittyBuzzBytecode = NULL;
static uint8_t g_bittyBuzzBytecodeLength = 0;


const uint8_t* bbz_bcodeFetcher(bbzpc_t offset, uint8_t size) {
    if (g_logger != NULL) {
        if (offset + size - 1>= g_bittyBuzzBytecodeLength) {
            g_logger->log(LogLevel::Error,
                          "BittyBuzz virtual machine requested out of bound bytecode");
        }
        if (size > 4) {
            g_logger->log(LogLevel::Warn,
                          "BittyBuzz requested more than 4 bytes from the bytecode array");
        }
    }

    return g_bittyBuzzBytecode + offset;
}

BittyBuzzBytecode::BittyBuzzBytecode(const ILogger& logger,
                                     const uint8_t* bytecode,
                                     const uint16_t bytecodeLength) :
    m_logger(logger) {
    g_logger = &logger;
    g_bittyBuzzBytecode = bytecode;
    g_bittyBuzzBytecodeLength = bytecodeLength;

    m_bytecode = bytecode;
    m_bytecodeSize = bytecodeLength;
}

bbzvm_bcode_fetch_fun BittyBuzzBytecode::getBytecodeFetchFunction() const {
    return bbz_bcodeFetcher;
}

uint16_t BittyBuzzBytecode::getBytecodeLength() const { return m_bytecodeSize; }
