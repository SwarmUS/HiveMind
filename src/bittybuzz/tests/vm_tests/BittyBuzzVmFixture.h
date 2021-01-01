
#ifndef __BITTYBUZZVMTESTFIXTURE_H_
#define __BITTYBUZZVMTESTFIXTURE_H_

#include "bittybuzz/BittyBuzzBytecode.h"
#include "bittybuzz/BittyBuzzVm.h"
#include "mocks/BSPInterfaceMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include <gtest/gtest.h>

class BittyBuzzVmTestFixture : public testing::Test {
  protected:
    BittyBuzzVm* m_bittybuzzVm;
    BittyBuzzBytecode* m_bittybuzzBytecode;

    LoggerInterfaceMock* m_loggerMock;
    BSPInterfaceMock* m_bspMock;

    int logCounter = 0;

    // Needs to be explicitly called since we use an overload
    void SetUp(const uint8_t* bytecode,
               const uint16_t bytecodeLength,
               uint16_t boardId,
               const FunctionRegister* functionRegisters,
               uint16_t lengthFunctionRegisters) {
        m_loggerMock = new LoggerInterfaceMock(logCounter);
        m_bspMock = new BSPInterfaceMock(boardId);
        m_bittybuzzBytecode = new BittyBuzzBytecode(*m_loggerMock, bytecode, bytecodeLength);
        m_bittybuzzVm = new BittyBuzzVm(*m_bittybuzzBytecode, *m_bspMock, *m_loggerMock,
                                        functionRegisters, lengthFunctionRegisters);
    }

    void TearDown() override {
        delete m_bittybuzzVm;
        delete m_bittybuzzBytecode;
        delete m_loggerMock;
        delete m_bspMock;
    }
};

#endif //__BITTYBUZZVMTESTFIXTURE_H_
