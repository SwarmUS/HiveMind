#include "BittyBuzzVmFixture.h"
#include "BittyBuzzVmTestsUtils.h"
#include "mocks/BittyBuzzClosureRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzMessageServiceInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include <stigmergy_get_bytecode.h>

TEST_F(BittyBuzzVmTestFixture, BittyBuzzVm_Stigmergy_get) {
    // Given
    uint16_t boardId = 1;
    BittyBuzzMessageHandlerInterfaceMock messageHandlerMock;
    BittyBuzzStringResolverInterfaceMock stringResolverMock;
    BittyBuzzClosureRegisterInterfaceMock closureRegisterMock;
    BittyBuyzzMessageServiceInterfaceMock messageServiceMock;

    EXPECT_CALL(messageHandlerMock, messageQueueLength).Times(1).WillOnce(testing::Return(0));

    std::array<UserFunctionRegister, 1> functionRegister = {{

        {BBZSTRID_assert_true, buzzAssertTrue}}};
    SetUp(bcode, bcode_size, boardId, &stringResolverMock, &messageHandlerMock,
          &closureRegisterMock, &messageServiceMock, functionRegister);

    // Then
    bbzmsg_payload_t bbzPayloadBuff;
    uint8_t buff[16] = {1, 2, 0, BBZSTRID_1, 0, 57, 42, 0, 1};
    bbzringbuf_construct(&bbzPayloadBuff, buff, 1, 16);
    bbzPayloadBuff.elsize = 1;
    bbzPayloadBuff.capacity = 8;
    bbzPayloadBuff.datastart = 0;
    bbzPayloadBuff.dataend = 9;

    bbzinmsg_queue_append(&bbzPayloadBuff);
    m_bittybuzzVm->step();

    // Expect
    EXPECT_EQ(logCounter, 1);
    EXPECT_EQ(vm->state, BBZVM_STATE_READY);
    EXPECT_EQ(vm->error, BBZVM_ERROR_NONE);
}
