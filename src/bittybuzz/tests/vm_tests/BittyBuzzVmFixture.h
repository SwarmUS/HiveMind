#ifndef __BITTYBUZZVMTESTFIXTURE_H_
#define __BITTYBUZZVMTESTFIXTURE_H_

#include "BittyBuzzVmTestsUtils.h"
#include "bittybuzz/BittyBuzzBytecode.h"
#include "bittybuzz/BittyBuzzVm.h"
#include "bittybuzz/IBittyBuzzMessageHandler.h"
#include "bittybuzz/IBittyBuzzMessageService.h"
#include "bittybuzz/IBittyBuzzNeighborsManager.h"
#include "bittybuzz/IBittyBuzzStringResolver.h"
#include "mocks/BSPInterfaceMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include "mocks/UserInterfaceMock.h"
#include <gtest/gtest.h>

class BittyBuzzVmTestFixture : public testing::Test {
  protected:
    BittyBuzzVm* m_bittybuzzVm;
    BittyBuzzBytecode* m_bittybuzzBytecode;
    LoggerInterfaceMock* m_loggerMock;
    BSPInterfaceMock* m_bspMock;
    UserInterfaceMock m_uiMock;

    int logCounter = 0;
    std::string logLastFormat;

    // Needs to be explicitly called since we use an overload
    template <typename Container>
    void SetUp(const uint8_t* bytecode,
               const uint16_t bytecodeLength,
               uint16_t boardId,
               IBittyBuzzStringResolver* bittyBuzzStringResolver,
               IBittyBuzzMessageHandler* bittyBuzzMessageHandler,
               IBittyBuzzClosureRegister* bittyBuzzClosureRegister,
               IBittyBuzzMessageService* bittyBuzzMessageService,
               IBittyBuzzNeighborsManager* m_neightborsManager,
               const Container& container) {

        g_assertTrueCallCount = 0;
        g_assertFalseCallCount = 0;

        m_loggerMock = new LoggerInterfaceMock(logCounter, logLastFormat);
        m_bspMock = new BSPInterfaceMock(boardId);
        m_bittybuzzBytecode = new BittyBuzzBytecode(*m_loggerMock, bytecode, bytecodeLength);

        m_bittybuzzVm = new BittyBuzzVm(*m_bittybuzzBytecode, *bittyBuzzStringResolver,
                                        *bittyBuzzMessageHandler, *bittyBuzzClosureRegister,
                                        *bittyBuzzMessageService, *m_neightborsManager, *m_bspMock,
                                        *m_loggerMock, m_uiMock, container);

        m_bittybuzzVm->init(container.data(), container.size());
    }

    void TearDown() override {
        delete m_bittybuzzVm;
        delete m_bittybuzzBytecode;
        delete m_loggerMock;
        delete m_bspMock;
    }
};

#endif //__BITTYBUZZVMTESTFIXTURE_H_
