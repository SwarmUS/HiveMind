#include "BittyBuzzVmFixture.h"
#include "BittyBuzzVmTestsUtils.h"
#include "mocks/BittyBuzzClosureRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzMessageServiceInterfaceMock.h"
#include "mocks/BittyBuzzNeighborsManagerInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include <bittybuzz/BittyBuzzLib.h>
#include <bittybuzz/BittyBuzzUIFunctions.h>
#include <user_interface_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_UserInterface) {
    // Given
    uint16_t boardId = 1;
    BittyBuzzMessageHandlerInterfaceMock messageHandlerMock;
    BittyBuzzStringResolverInterfaceMock stringResolverMock;
    BittyBuzzClosureRegisterInterfaceMock closureRegisterMock;
    BittyBuyzzMessageServiceInterfaceMock messageServiceMock;
    BittyBuzzNeighborsManagerInterfaceMock neighborsManagerMock;

    EXPECT_CALL(neighborsManagerMock, updateNeighbors).Times(1);
    EXPECT_CALL(messageHandlerMock, messageQueueLength).Times(1).WillOnce(testing::Return(0));

    std::array<BittyBuzzLibMemberRegister, 2> uiLibRegisters = {
        {{BBZSTRID_set_led, BittyBuzzUIFunctions::setLed},
         {BBZSTRID_set_hex, BittyBuzzUIFunctions::setHex}}};
    BittyBuzzLib uiLib(BBZSTRID_ui, uiLibRegisters);

    std::vector<std::reference_wrapper<IBittyBuzzLib>> libraries;
    libraries.emplace_back(uiLib);

    SetUp(bcode, bcode_size, boardId, &stringResolverMock, &messageHandlerMock,
          &closureRegisterMock, &messageServiceMock, &neighborsManagerMock, libraries);

    EXPECT_CALL(m_userUIMock, setLed(true)).Times(1);
    EXPECT_CALL(m_userUIMock, setLed(false)).Times(3);
    EXPECT_CALL(m_userUIMock, setSegment(UserSegment::One)).Times(1);
    EXPECT_CALL(m_userUIMock, setSegment(UserSegment::F)).Times(1);
    EXPECT_CALL(m_userUIMock, setSegment(UserSegment::Zero)).Times(1);

    // Then
    BBVMRet ret = m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(ret, BBVMRet::Ok);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
