#include "mocks/BSPInterfaceMock.h"
#include "mocks/CircularQueueInterfaceMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include <array>
#include <bittybuzz/BittyBuzzLib.h>
#include <bittybuzz/BittyBuzzMessageService.h>
#include <gtest/gtest.h>

class BittyBuzzMessageServiceTestFixture : public testing::Test {
  protected:
    BittyBuzzMessageService* m_messageService;

    CircularQueueInterfaceMock<MessageDTO> m_hostOutputQueueMock;
    CircularQueueInterfaceMock<MessageDTO> m_remoteOutputQueueMock;
    LoggerInterfaceMock* m_loggerMock;
    BSPInterfaceMock* m_bspMock;
    int m_logCounter = 0;
    const uint16_t m_bspUUID = 42;
    const uint16_t m_remoteUUID = 43;
    const int64_t m_argVal = 42;
    const std::string m_functionName = "print";
    std::string m_logLastFormat;

    // DTO
    std::array<FunctionCallArgumentDTO, 1> m_arguments = {{m_argVal}};

    void SetUp() override {
        m_loggerMock = new LoggerInterfaceMock(m_logCounter, m_logLastFormat);
        m_bspMock = new BSPInterfaceMock(m_bspUUID);
        m_messageService = new BittyBuzzMessageService(
            m_hostOutputQueueMock, m_remoteOutputQueueMock, *m_bspMock, *m_loggerMock);
    }

    void TearDown() override {
        delete m_loggerMock;
        delete m_bspMock;
        delete m_messageService;
    }
};

TEST_F(BittyBuzzMessageServiceTestFixture, BittyBuzzMessageService_callHostFunction_remote_valid) {
    // Given
    MessageDTO messageSent;
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(true)));

    // Then
    bool ret = m_messageService->callHostFunction(m_remoteUUID, m_functionName.c_str(),
                                                  m_arguments.data(), m_arguments.size());

    // Expect
    auto req = std::get<RequestDTO>(messageSent.getMessage());
    auto ureq = std::get<UserCallRequestDTO>(req.getRequest());
    auto freq = std::get<FunctionCallRequestDTO>(ureq.getRequest());
    auto arg = std::get<int64_t>(freq.getArguments()[0].getArgument());
    EXPECT_TRUE(ret);
    EXPECT_EQ(freq.getArgumentsLength(), m_arguments.size());
    EXPECT_EQ(arg, m_argVal);
    EXPECT_STREQ(freq.getFunctionName(), m_functionName.c_str());
}

TEST_F(BittyBuzzMessageServiceTestFixture,
       BittyBuzzMessageService_callHostFunction_remote_failToPush) {
    // Given
    MessageDTO messageSent;
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(false)));

    // Then
    bool ret = m_messageService->callHostFunction(m_remoteUUID, m_functionName.c_str(),
                                                  m_arguments.data(), m_arguments.size());

    // Expect
    auto req = std::get<RequestDTO>(messageSent.getMessage());
    auto ureq = std::get<UserCallRequestDTO>(req.getRequest());
    auto freq = std::get<FunctionCallRequestDTO>(ureq.getRequest());
    auto arg = std::get<int64_t>(freq.getArguments()[0].getArgument());
    EXPECT_FALSE(ret);
    EXPECT_EQ(freq.getArgumentsLength(), m_arguments.size());
    EXPECT_EQ(arg, m_argVal);
    EXPECT_STREQ(freq.getFunctionName(), m_functionName.c_str());
}

TEST_F(BittyBuzzMessageServiceTestFixture, BittyBuzzMessageService_callHostFunction_host_valid) {
    // Given
    MessageDTO messageSent;
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(true)));

    // Then
    bool ret = m_messageService->callHostFunction(m_bspUUID, m_functionName.c_str(),
                                                  m_arguments.data(), m_arguments.size());

    // Expect
    auto req = std::get<RequestDTO>(messageSent.getMessage());
    auto ureq = std::get<UserCallRequestDTO>(req.getRequest());
    auto freq = std::get<FunctionCallRequestDTO>(ureq.getRequest());
    auto arg = std::get<int64_t>(freq.getArguments()[0].getArgument());
    EXPECT_TRUE(ret);
    EXPECT_EQ(freq.getArgumentsLength(), m_arguments.size());
    EXPECT_EQ(arg, m_argVal);
    EXPECT_STREQ(freq.getFunctionName(), m_functionName.c_str());
}

TEST_F(BittyBuzzMessageServiceTestFixture,
       BittyBuzzMessageService_callHostFunction_host_failToPush) {
    // Given
    MessageDTO messageSent;
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(false)));

    // Then
    bool ret = m_messageService->callHostFunction(m_bspUUID, m_functionName.c_str(),
                                                  m_arguments.data(), m_arguments.size());

    // Expect
    auto req = std::get<RequestDTO>(messageSent.getMessage());
    auto ureq = std::get<UserCallRequestDTO>(req.getRequest());
    auto freq = std::get<FunctionCallRequestDTO>(ureq.getRequest());
    auto arg = std::get<int64_t>(freq.getArguments()[0].getArgument());
    EXPECT_FALSE(ret);
    EXPECT_EQ(freq.getArgumentsLength(), m_arguments.size());
    EXPECT_EQ(arg, m_argVal);
    EXPECT_STREQ(freq.getFunctionName(), m_functionName.c_str());
}

TEST_F(BittyBuzzMessageServiceTestFixture,
       BittyBuzzMessageService_callHostFunction_broadcast_valid) {
    // Given
    MessageDTO messageSentHost;
    MessageDTO messageSentRemote;
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSentRemote), testing::Return(true)));
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSentHost), testing::Return(true)));

    // Then
    bool ret = m_messageService->callHostFunction(0, m_functionName.c_str(), m_arguments.data(),
                                                  m_arguments.size());

    // Expect

    // Remote
    auto reqRemote = std::get<RequestDTO>(messageSentRemote.getMessage());
    auto ureqRemote = std::get<UserCallRequestDTO>(reqRemote.getRequest());
    auto freqRemote = std::get<FunctionCallRequestDTO>(ureqRemote.getRequest());
    auto argRemote = std::get<int64_t>(freqRemote.getArguments()[0].getArgument());

    // Host
    auto reqHost = std::get<RequestDTO>(messageSentHost.getMessage());
    auto ureqHost = std::get<UserCallRequestDTO>(reqHost.getRequest());
    auto freqHost = std::get<FunctionCallRequestDTO>(ureqHost.getRequest());
    auto argHost = std::get<int64_t>(freqHost.getArguments()[0].getArgument());

    EXPECT_TRUE(ret);
    // Remote
    EXPECT_EQ(freqRemote.getArgumentsLength(), m_arguments.size());
    EXPECT_EQ(argRemote, m_argVal);
    EXPECT_STREQ(freqRemote.getFunctionName(), m_functionName.c_str());
    // Host
    EXPECT_EQ(freqHost.getArgumentsLength(), m_arguments.size());
    EXPECT_EQ(argHost, m_argVal);
    EXPECT_STREQ(freqHost.getFunctionName(), m_functionName.c_str());
}

TEST_F(BittyBuzzMessageServiceTestFixture,
       BittyBuzzMessageService_callHostFunction_broadcast_hostPushFail) {
    // Given
    MessageDTO messageSentHost;
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_))
        .Times(testing::AtMost(1))
        .WillRepeatedly(testing::Return(true));
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).Times(1).WillOnce(testing::Return(false));

    // Then
    bool ret = m_messageService->callHostFunction(0, m_functionName.c_str(), m_arguments.data(),
                                                  m_arguments.size());

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(BittyBuzzMessageServiceTestFixture,
       BittyBuzzMessageService_callHostFunction_broadcast_remotePushFail) {
    // Given
    MessageDTO messageSentHost;
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::Return(false));
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_))
        .Times(testing::AtMost(1))
        .WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageService->callHostFunction(0, m_functionName.c_str(), m_arguments.data(),
                                                  m_arguments.size());

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(BittyBuzzMessageServiceTestFixture,
       BittyBuzzMessageService_callHostFunction_broadcast_bothPushFail) {
    // Given
    MessageDTO messageSentHost;
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_)).WillOnce(testing::Return(false));
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageService->callHostFunction(0, m_functionName.c_str(), m_arguments.data(),
                                                  m_arguments.size());

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(BittyBuzzMessageServiceTestFixture, BittyBuzzMessageService_sendBuzzMessage_pushValid) {
    // Given
    BuzzMessageDTO buzzMsg(NULL, 0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_)).WillOnce(testing::Return(true));
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).Times(0);

    // Then
    bool ret = m_messageService->sendBuzzMessage(buzzMsg);

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(BittyBuzzMessageServiceTestFixture, BittyBuzzMessageService_sendBuzzMessage_pushInvalid) {
    // Given
    BuzzMessageDTO buzzMsg(NULL, 0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_)).WillOnce(testing::Return(false));
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).Times(0);

    // Then
    bool ret = m_messageService->sendBuzzMessage(buzzMsg);

    // Expect
    EXPECT_FALSE(ret);
}
