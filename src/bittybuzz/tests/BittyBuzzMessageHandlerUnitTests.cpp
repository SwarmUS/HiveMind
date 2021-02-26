
#include "mocks/BSPInterfaceMock.h"
#include "mocks/BittyBuzzBytecodeInterfaceMock.h"
#include "mocks/BittyBuzzFunctionRegisterInterfaceMock.h"
#include "mocks/BittyBuzzMessageHandlerInterfaceMock.h"
#include "mocks/BittyBuzzStringResolverInterfaceMock.h"
#include "mocks/CircularQueueInterfaceMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include <bittybuzz/BittyBuzzMessageHandler.h>
#include <bittybuzz/BittyBuzzVm.h>
#include <gtest/gtest.h>

const uint8_t* mockbcodeFetcher(bbzpc_t offset, uint8_t size) {
    (void)offset;
    (void)size;
    static uint8_t s_data[4] = {BBZVM_INSTR_NOP, BBZVM_INSTR_NOP, BBZVM_INSTR_NOP, BBZVM_INSTR_NOP};
    return s_data;
}

class BittyBuzzMessageHandlerFixture : public testing::Test {
  protected:
    int m_logCounter;
    std::string m_logLastFormat;
    const uint16_t m_uuid = 42;
    const uint16_t m_srcUuid = 43;

    LoggerInterfaceMock* m_loggerMock;
    BittyBuzzFunctionRegisterInterfaceMock m_functionRegisterMock;
    BittyBuzzMessageHandlerInterfaceMock m_messageHandlerMock;
    CircularQueueInterfaceMock<MessageDTO> m_inputQueueMock;
    CircularQueueInterfaceMock<MessageDTO> m_hostOutputQueueMock;
    CircularQueueInterfaceMock<MessageDTO> m_remoteOutputQueueMock;

    BittyBuzzVm* m_bbzVM;
    BittyBuzzMessageHandler* m_bbzMessageHandler;

    // Messages
    std::string m_functionName = "print";
    FunctionCallRequestDTO* m_fRequest;
    UserCallRequestDTO* m_uRequest;
    RequestDTO* m_request;

    FunctionCallResponseDTO* m_fResponse;
    UserCallResponseDTO* m_uResponse;
    ResponseDTO* m_response;

    // VM setup
    BittyBuzzVm* m_bittybuzzVmMock;
    BSPInterfaceMock* m_bspMock;
    BittyBuzzStringResolverInterfaceMock m_bittyBuzzStringResolverMock;
    BittyBuzzBytecodeInterfaceMock m_bittybuzzBytecode;

    void SetUp() override {
        // Message
        m_fRequest = new FunctionCallRequestDTO(m_functionName.c_str(), NULL, 0);
        m_uRequest =
            new UserCallRequestDTO(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, *m_fRequest);
        m_request = new RequestDTO(1, *m_uRequest);

        m_fResponse = new FunctionCallResponseDTO(GenericResponseStatusDTO::Ok, "");
        m_uResponse =
            new UserCallResponseDTO(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, *m_fResponse);
        m_response = new ResponseDTO(1, *m_uResponse);

        // Logger
        m_loggerMock = new LoggerInterfaceMock(m_logCounter, m_logLastFormat);

        // VM setup, needs to be instanciated so we can do operations on the VM
        // I though about using an interface for the VM, but an interface for the WHOLE bittybuzz
        // api is a bit much
        EXPECT_CALL(m_bittybuzzBytecode, getBytecodeLength).WillRepeatedly(testing::Return(0));
        EXPECT_CALL(m_bittybuzzBytecode, getBytecodeFetchFunction)
            .WillRepeatedly(testing::Return(mockbcodeFetcher));
        m_bspMock = new BSPInterfaceMock(m_uuid);
        m_bittybuzzVmMock = new BittyBuzzVm(
            m_bittybuzzBytecode, m_bittyBuzzStringResolverMock, m_messageHandlerMock,
            m_functionRegisterMock, *m_bspMock, *m_loggerMock, std::array<FunctionRegister, 0>{});
        // Message Handler
        m_bbzMessageHandler = new BittyBuzzMessageHandler(
            m_functionRegisterMock, m_inputQueueMock, m_hostOutputQueueMock,
            m_remoteOutputQueueMock, m_uuid, *m_loggerMock);
    }

    void TearDown() override {
        delete m_bbzMessageHandler;

        delete m_bspMock;
        delete m_bittybuzzVmMock;

        delete m_loggerMock;

        delete m_request;
        delete m_uRequest;
        delete m_fRequest;

        delete m_fResponse;
        delete m_uResponse;
        delete m_response;
    }
};
/*
 * Message Queue length
 * */

TEST_F(BittyBuzzMessageHandlerFixture, BittyBuzzMessageHandler_messageQueueLength_0) {
    // Given
    uint16_t queueLength = 0;
    EXPECT_CALL(m_inputQueueMock, getLength).Times(1).WillOnce(testing::Return(queueLength));

    // Then
    uint16_t ret = m_bbzMessageHandler->messageQueueLength();

    // Expect
    EXPECT_EQ(ret, queueLength);
}

TEST_F(BittyBuzzMessageHandlerFixture, BittyBuzzMessageHandler_messageQueueLength_42) {
    // Given
    uint16_t queueLength = 42;
    EXPECT_CALL(m_inputQueueMock, getLength).Times(1).WillOnce(testing::Return(queueLength));

    // Then
    uint16_t ret = m_bbzMessageHandler->messageQueueLength();

    // Expect
    EXPECT_EQ(ret, queueLength);
}
/*
** REQUESTS
*/

/*
** Requests, send to host
*/

TEST_F(BittyBuzzMessageHandlerFixture,
       BittyBuzzMessageHandler_processMessage_validFunctionCall_host_pushSuccessful) {
    // Given
    MessageDTO message = MessageDTO(m_uuid, m_uuid, *m_request);
    MessageDTO messageSent;
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;
    std::optional<bbzheap_idx_t> functionHeapIdx = 1;

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_))
        .Times(1)
        .WillOnce(testing::Return(functionHeapIdx));

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(true)));

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    auto resp = std::get<ResponseDTO>(messageSent.getMessage());
    auto uresp = std::get<UserCallResponseDTO>(resp.getResponse());
    auto fresp = std::get<FunctionCallResponseDTO>(uresp.getResponse());
    EXPECT_TRUE(ret);
    EXPECT_EQ(fresp.getResponse().getStatus(), GenericResponseStatusDTO::Ok);
}

TEST_F(BittyBuzzMessageHandlerFixture,
       BittyBuzzMessageHandler_processMessage_validFunctionCall_host_pushunSuccessful) {
    // Given
    MessageDTO message = MessageDTO(m_uuid, m_uuid, *m_request);
    MessageDTO messageSent;
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;
    std::optional<bbzheap_idx_t> functionHeapIdx = 1;

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_))
        .Times(1)
        .WillOnce(testing::Return(functionHeapIdx));

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(false)));

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    auto resp = std::get<ResponseDTO>(messageSent.getMessage());
    auto uresp = std::get<UserCallResponseDTO>(resp.getResponse());
    auto fresp = std::get<FunctionCallResponseDTO>(uresp.getResponse());
    EXPECT_FALSE(ret);
    EXPECT_EQ(fresp.getResponse().getStatus(), GenericResponseStatusDTO::Ok);
}

TEST_F(
    BittyBuzzMessageHandlerFixture,
    BittyBuzzMessageHandler_processMessage_invalidFunctionCall_functionNotFound_host_pushSuccessful) {
    // Given
    MessageDTO message = MessageDTO(m_uuid, m_uuid, *m_request);
    MessageDTO messageSent;
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;
    std::optional<bbzheap_idx_t> functionHeapIdx = {};

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_))
        .Times(1)
        .WillOnce(testing::Return(functionHeapIdx));

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(true)));

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    auto resp = std::get<ResponseDTO>(messageSent.getMessage());
    auto uresp = std::get<UserCallResponseDTO>(resp.getResponse());
    auto fresp = std::get<FunctionCallResponseDTO>(uresp.getResponse());
    EXPECT_TRUE(ret);
    EXPECT_EQ(fresp.getResponse().getStatus(), GenericResponseStatusDTO::BadRequest);
}

TEST_F(
    BittyBuzzMessageHandlerFixture,
    BittyBuzzMessageHandler_processMessage_invalidFunctionCall_functionNotFound_host_pushUnsuccessful) {
    // Given
    MessageDTO message = MessageDTO(m_uuid, m_uuid, *m_request);
    MessageDTO messageSent;
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;
    std::optional<bbzheap_idx_t> functionHeapIdx = {};

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_))
        .Times(1)
        .WillOnce(testing::Return(functionHeapIdx));

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(false)));

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    auto resp = std::get<ResponseDTO>(messageSent.getMessage());
    auto uresp = std::get<UserCallResponseDTO>(resp.getResponse());
    auto fresp = std::get<FunctionCallResponseDTO>(uresp.getResponse());
    EXPECT_FALSE(ret);
    EXPECT_EQ(fresp.getResponse().getStatus(), GenericResponseStatusDTO::BadRequest);
}

TEST_F(BittyBuzzMessageHandlerFixture,
       BittyBuzzMessageHandler_processMessage_invalidUserCall_host_pushSuccessful) {
    // Given
    m_uRequest->setRequest(std::monostate());
    m_request->setRequest(*m_uRequest);
    MessageDTO message = MessageDTO(m_uuid, m_uuid, *m_request);
    MessageDTO messageSent;
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_)).Times(0);

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(true)));

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    auto resp = std::get<ResponseDTO>(messageSent.getMessage());
    auto uresp = std::get<UserCallResponseDTO>(resp.getResponse());
    auto gresp = std::get<GenericResponseDTO>(uresp.getResponse());
    EXPECT_TRUE(ret);
    EXPECT_EQ(gresp.getStatus(), GenericResponseStatusDTO::BadRequest);
}

TEST_F(BittyBuzzMessageHandlerFixture,
       BittyBuzzMessageHandler_processMessage_invalidUserCall_host_pushUnsuccessful) {
    // Given
    m_uRequest->setRequest(std::monostate());
    m_request->setRequest(*m_uRequest);
    MessageDTO message = MessageDTO(m_uuid, m_uuid, *m_request);
    MessageDTO messageSent;
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_)).Times(0);

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(false)));

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    auto resp = std::get<ResponseDTO>(messageSent.getMessage());
    auto uresp = std::get<UserCallResponseDTO>(resp.getResponse());
    auto gresp = std::get<GenericResponseDTO>(uresp.getResponse());
    EXPECT_FALSE(ret);
    EXPECT_EQ(gresp.getStatus(), GenericResponseStatusDTO::BadRequest);
}

TEST_F(BittyBuzzMessageHandlerFixture,
       BittyBuzzMessageHandler_processMessage_invalidRequest_host_pushSuccessful) {
    // Given
    m_request->setRequest(std::monostate());
    MessageDTO message = MessageDTO(m_uuid, m_uuid, *m_request);
    MessageDTO messageSent;
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_)).Times(0);

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(true)));

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    auto resp = std::get<ResponseDTO>(messageSent.getMessage());
    auto gresp = std::get<GenericResponseDTO>(resp.getResponse());
    EXPECT_TRUE(ret);
    EXPECT_EQ(gresp.getStatus(), GenericResponseStatusDTO::BadRequest);
}

TEST_F(BittyBuzzMessageHandlerFixture,
       BittyBuzzMessageHandler_processMessage_invalidRequest_host_pushUnsuccessful) {
    // Given
    m_request->setRequest(std::monostate());
    MessageDTO message = MessageDTO(m_uuid, m_uuid, *m_request);
    MessageDTO messageSent;
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_)).Times(0);

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(false)));

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    auto resp = std::get<ResponseDTO>(messageSent.getMessage());
    auto gresp = std::get<GenericResponseDTO>(resp.getResponse());
    EXPECT_FALSE(ret);
    EXPECT_EQ(gresp.getStatus(), GenericResponseStatusDTO::BadRequest);
}

/*
** Requests, send to remote
*/
TEST_F(BittyBuzzMessageHandlerFixture,
       BittyBuzzMessageHandler_processMessage_validFunctionCall_remote_pushSuccessful) {
    // Given
    MessageDTO message = MessageDTO(m_srcUuid, m_uuid, *m_request);
    MessageDTO messageSent;
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;
    std::optional<bbzheap_idx_t> functionHeapIdx = 1;

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_))
        .Times(1)
        .WillOnce(testing::Return(functionHeapIdx));

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(true)));

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    auto resp = std::get<ResponseDTO>(messageSent.getMessage());
    auto uresp = std::get<UserCallResponseDTO>(resp.getResponse());
    auto fresp = std::get<FunctionCallResponseDTO>(uresp.getResponse());
    EXPECT_TRUE(ret);
    EXPECT_EQ(fresp.getResponse().getStatus(), GenericResponseStatusDTO::Ok);
}

TEST_F(BittyBuzzMessageHandlerFixture,
       BittyBuzzMessageHandler_processMessage_validFunctionCall_remote_pushunSuccessful) {
    // Given
    MessageDTO message = MessageDTO(m_srcUuid, m_uuid, *m_request);
    MessageDTO messageSent;
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;
    std::optional<bbzheap_idx_t> functionHeapIdx = 1;

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_))
        .Times(1)
        .WillOnce(testing::Return(functionHeapIdx));

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(false)));

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    auto resp = std::get<ResponseDTO>(messageSent.getMessage());
    auto uresp = std::get<UserCallResponseDTO>(resp.getResponse());
    auto fresp = std::get<FunctionCallResponseDTO>(uresp.getResponse());
    EXPECT_FALSE(ret);
    EXPECT_EQ(fresp.getResponse().getStatus(), GenericResponseStatusDTO::Ok);
}

TEST_F(
    BittyBuzzMessageHandlerFixture,
    BittyBuzzMessageHandler_processMessage_invalidFunctionCall_functionNotFound_remote_pushSuccessful) {
    // Given
    MessageDTO message = MessageDTO(m_srcUuid, m_uuid, *m_request);
    MessageDTO messageSent;
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;
    std::optional<bbzheap_idx_t> functionHeapIdx = {};

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_))
        .Times(1)
        .WillOnce(testing::Return(functionHeapIdx));

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(true)));

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    auto resp = std::get<ResponseDTO>(messageSent.getMessage());
    auto uresp = std::get<UserCallResponseDTO>(resp.getResponse());
    auto fresp = std::get<FunctionCallResponseDTO>(uresp.getResponse());
    EXPECT_TRUE(ret);
    EXPECT_EQ(fresp.getResponse().getStatus(), GenericResponseStatusDTO::BadRequest);
}

TEST_F(
    BittyBuzzMessageHandlerFixture,
    BittyBuzzMessageHandler_processMessage_invalidFunctionCall_functionNotFound_remote_pushUnsuccessful) {
    // Given
    MessageDTO message = MessageDTO(m_srcUuid, m_uuid, *m_request);
    MessageDTO messageSent;
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;
    std::optional<bbzheap_idx_t> functionHeapIdx = {};

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_))
        .Times(1)
        .WillOnce(testing::Return(functionHeapIdx));

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(false)));

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    auto resp = std::get<ResponseDTO>(messageSent.getMessage());
    auto uresp = std::get<UserCallResponseDTO>(resp.getResponse());
    auto fresp = std::get<FunctionCallResponseDTO>(uresp.getResponse());
    EXPECT_FALSE(ret);
    EXPECT_EQ(fresp.getResponse().getStatus(), GenericResponseStatusDTO::BadRequest);
}

TEST_F(BittyBuzzMessageHandlerFixture,
       BittyBuzzMessageHandler_processMessage_invalidUserCall_remote_pushSuccessful) {
    // Given
    m_uRequest->setRequest(std::monostate());
    m_request->setRequest(*m_uRequest);
    MessageDTO message = MessageDTO(m_srcUuid, m_uuid, *m_request);
    MessageDTO messageSent;
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_)).Times(0);

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(true)));

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    auto resp = std::get<ResponseDTO>(messageSent.getMessage());
    auto uresp = std::get<UserCallResponseDTO>(resp.getResponse());
    auto gresp = std::get<GenericResponseDTO>(uresp.getResponse());
    EXPECT_TRUE(ret);
    EXPECT_EQ(gresp.getStatus(), GenericResponseStatusDTO::BadRequest);
}

TEST_F(BittyBuzzMessageHandlerFixture,
       BittyBuzzMessageHandler_processMessage_invalidUserCall_remote_pushUnsuccessful) {
    // Given
    m_uRequest->setRequest(std::monostate());
    m_request->setRequest(*m_uRequest);
    MessageDTO message = MessageDTO(m_srcUuid, m_uuid, *m_request);
    MessageDTO messageSent;
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_)).Times(0);

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(false)));

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    auto resp = std::get<ResponseDTO>(messageSent.getMessage());
    auto uresp = std::get<UserCallResponseDTO>(resp.getResponse());
    auto gresp = std::get<GenericResponseDTO>(uresp.getResponse());
    EXPECT_FALSE(ret);
    EXPECT_EQ(gresp.getStatus(), GenericResponseStatusDTO::BadRequest);
}

TEST_F(BittyBuzzMessageHandlerFixture,
       BittyBuzzMessageHandler_processMessage_invalidRequest_remote_pushSuccessful) {
    // Given
    m_request->setRequest(std::monostate());
    MessageDTO message = MessageDTO(m_srcUuid, m_uuid, *m_request);
    MessageDTO messageSent;
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_)).Times(0);

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(true)));

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    auto resp = std::get<ResponseDTO>(messageSent.getMessage());
    auto gresp = std::get<GenericResponseDTO>(resp.getResponse());
    EXPECT_TRUE(ret);
    EXPECT_EQ(gresp.getStatus(), GenericResponseStatusDTO::BadRequest);
}

TEST_F(BittyBuzzMessageHandlerFixture,
       BittyBuzzMessageHandler_processMessage_invalidRequest_remote_pushUnsuccessful) {
    // Given
    m_request->setRequest(std::monostate());
    MessageDTO message = MessageDTO(m_srcUuid, m_uuid, *m_request);
    MessageDTO messageSent;
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_)).Times(0);

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(false)));

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    auto resp = std::get<ResponseDTO>(messageSent.getMessage());
    auto gresp = std::get<GenericResponseDTO>(resp.getResponse());
    EXPECT_FALSE(ret);
    EXPECT_EQ(gresp.getStatus(), GenericResponseStatusDTO::BadRequest);
}

/*
** RESPONSE
*/

TEST_F(BittyBuzzMessageHandlerFixture,
       BittyBuzzMessageHandler_processMessage_validUserCallResponse) {
    // Given
    MessageDTO message = MessageDTO(m_srcUuid, m_uuid, *m_response);
    MessageDTO messageSent;
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_)).Times(0);

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_)).Times(0);

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(BittyBuzzMessageHandlerFixture,
       BittyBuzzMessageHandler_processMessage_validGenericResponse) {
    // Given
    m_response->setResponse(GenericResponseDTO(GenericResponseStatusDTO::Ok, ""));
    MessageDTO message = MessageDTO(m_srcUuid, m_uuid, *m_response);
    MessageDTO messageSent;
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_)).Times(0);

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_)).Times(0);

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(BittyBuzzMessageHandlerFixture,
       BittyBuzzMessageHandler_processMessage_invalidUnkownResponse) {
    // Given
    m_response->setResponse(std::monostate());
    MessageDTO message = MessageDTO(m_srcUuid, m_uuid, *m_response);
    MessageDTO messageSent;
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_)).Times(0);

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_)).Times(0);

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    EXPECT_FALSE(ret);
}

/*
** MESSAGES
*/
TEST_F(BittyBuzzMessageHandlerFixture, BittyBuzzMessageHandler_processMessage_invalidMessage) {
    // Given
    MessageDTO message = MessageDTO(m_srcUuid, m_uuid, *m_request);
    message.setMessage(std::monostate());
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_)).Times(0);

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_)).Times(0);

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(BittyBuzzMessageHandlerFixture, BittyBuzzMessageHandler_processMessage_noMessage) {
    // Given
    MessageDTO message = MessageDTO(m_srcUuid, m_uuid, *m_request);
    message.setMessage(std::monostate());
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = {};

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_functionRegisterMock, getFunctionHeapIdx(testing::_)).Times(0);

    EXPECT_CALL(m_inputQueueMock, pop).Times(0);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_)).Times(0);

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    EXPECT_TRUE(ret);
}
