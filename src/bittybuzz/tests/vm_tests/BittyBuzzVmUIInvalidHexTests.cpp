#include "BittyBuzzVmFixture.h"
#include "BittyBuzzVmTestsUtils.h"
#include "mocks/BittyBuzzClosureRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzMessageServiceInterfaceMock.h"
#include "mocks/BittyBuzzNeighborsManagerInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include <bittybuzz/BittyBuzzLib.h>
#include <bittybuzz/BittyBuzzUIFunctions.h>
#include <user_interface_invalid_hex_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_UserInterface_invalidHex) {
    // Given
    uint16_t boardId = 1;
    BittyBuzzMessageHandlerInterfaceMock messageHandlerMock;
    BittyBuzzStringResolverInterfaceMock stringResolverMock;
    BittyBuzzClosureRegisterInterfaceMock closureRegisterMock;
    BittyBuyzzMessageServiceInterfaceMock messageServiceMock;
    BittyBuzzNeighborsManagerInterfaceMock neighborsManagerMock;

    EXPECT_CALL(neighborsManagerMock, updateNeighbors).Times(1);
    EXPECT_CALL(messageHandlerMock, messageQueueLength).Times(1).WillOnce(testing::Return(0));
    EXPECT_CALL(stringResolverMock, getString).WillRepeatedly(testing::Return(""));

    std::array<BittyBuzzLibMemberRegister, 1> uiLibRegisters = {
        {{BBZSTRID_set_hex, BittyBuzzUIFunctions::setHex}}};
    BittyBuzzLib uiLib(BBZSTRID_ui, uiLibRegisters);

    std::vector<std::reference_wrapper<IBittyBuzzLib>> libraries;
    libraries.emplace_back(uiLib);

    SetUp(bcode, bcode_size, boardId, &stringResolverMock, &messageHandlerMock,
          &closureRegisterMock, &messageServiceMock, &neighborsManagerMock, libraries);

    EXPECT_CALL(m_userUIMock, setSegment).Times(0);

    // Then
    BBVMRet ret = m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(ret, BBVMRet::VmErr);
    EXPECT_EQ(vm->state, BBZVM_STATE_ERROR);
    EXPECT_EQ(vm->error, BBZVM_ERROR_TYPE);
}
