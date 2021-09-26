#include "BittyBuzzVmFixture.h"
#include "BittyBuzzVmTestsUtils.h"
#include "mocks/BittyBuzzClosureRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzMessageServiceInterfaceMock.h"
#include "mocks/BittyBuzzNeighborsManagerInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include <bittybuzz/BittyBuzzLib.h>
#include <bittybuzz/BittyBuzzMathFunctions.h>
#include <bittybuzz/BittyBuzzUserFunctions.h>
#include <gmock/gmock.h>
#include <table_library_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_tableLibrary) {
    // Given
    uint16_t boardId = 42;
    BittyBuzzMessageHandlerInterfaceMock messageHandlerMock;
    BittyBuzzStringResolverInterfaceMock stringResolverMock;
    BittyBuzzClosureRegisterInterfaceMock closureRegisterMock;
    BittyBuyzzMessageServiceInterfaceMock messageServiceMock;
    BittyBuzzNeighborsManagerInterfaceMock neighborsManagerMock;

    bbzvm_funp methodGet10 = []() {
        bbzvm_assert_lnum(0); // NOLINT
        bbzvm_pushi(10);
        bbzvm_ret1();
    };

    EXPECT_CALL(neighborsManagerMock, updateNeighbors).Times(1);
    EXPECT_CALL(messageHandlerMock, messageQueueLength).Times(1).WillOnce(testing::Return(0));
    EXPECT_CALL(messageServiceMock, queueBuzzMessages).WillOnce(testing::Return(true));

    std::array<BittyBuzzLibMemberRegister, 1> functionRegisters = {
        {{BBZSTRID_assert_true, buzzAssertTrue}}};

    std::array<BittyBuzzLibMemberRegister, 2> tableLibRegister = {
        {{BBZSTRID_get_10, methodGet10}, {BBZSTRID_value_5, (int16_t)5}}};

    BittyBuzzLib globalLib(functionRegisters);
    BittyBuzzLib tableLib(BBZSTRID_lib, tableLibRegister);

    std::vector<std::reference_wrapper<IBittyBuzzLib>> libraries;
    libraries.emplace_back(globalLib);
    libraries.emplace_back(tableLib);

    SetUp(bcode, bcode_size, boardId, &stringResolverMock, &messageHandlerMock,
          &closureRegisterMock, &messageServiceMock, &neighborsManagerMock, libraries);

    // Then
    m_bittybuzzVm->step();

    // Expect

    EXPECT_EQ(g_assertTrueCallCount, 1);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
