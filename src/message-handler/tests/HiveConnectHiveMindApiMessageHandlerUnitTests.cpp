
#include "mocks/BSPInterfaceMock.h"
#include "mocks/CircularQueueInterfaceMock.h"
#include "mocks/InterlocInterfaceMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include "utils/DTOMatchers.h"
#include <gtest/gtest.h>
#include <message-handler/HiveConnectHiveMindApiMessageHandler.h>
#include <vector>

class HiveConnectHiveMindApiMessageHandlerFixture : public testing::Test {
  protected:
    LoggerInterfaceMock* m_loggerInterfaceMock;
    HiveConnectHiveMindApiMessageHandler* m_hivemindApiReqHandler;
    testing::StrictMock<CircularQueueInterfaceMock<MessageDTO>> m_hostQueueMock;
    testing::StrictMock<CircularQueueInterfaceMock<MessageDTO>> m_remoteQueueMock;

    const uint16_t m_requestId = 42;
    int m_logCallCounter;
    std::string m_logLastFormat;

    void SetUp() override {
        m_loggerInterfaceMock = new LoggerInterfaceMock();
        m_hivemindApiReqHandler = new HiveConnectHiveMindApiMessageHandler(
            m_hostQueueMock, m_remoteQueueMock, *m_loggerInterfaceMock);
    }
    void TearDown() override {
        delete m_hivemindApiReqHandler;
        delete m_loggerInterfaceMock;
    }
};

TEST_F(HiveConnectHiveMindApiMessageHandlerFixture,
       HiveConnectHiveMindApiMessageHandler_handleMessage_agentResponse_valid) {
    // Given
    MessageDTO messageSent;
    HiveConnectHiveMindApiDTO req(m_requestId, GetAgentsListResponseDTO(NULL, 0));
    EXPECT_CALL(m_hostQueueMock, push(testing::_))
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(true)));

    // Then
    bool ret = m_hivemindApiReqHandler->handleMessage(1, 1, req);

    // Expect
    EXPECT_TRUE(ret);
    EXPECT_TRUE(std::holds_alternative<ResponseDTO>(messageSent.getMessage()));
}

TEST_F(HiveConnectHiveMindApiMessageHandlerFixture,
       HiveConnectHiveMindApiMessageHandler_handleMessage_agentResponse_invalid) {
    // Given
    HiveConnectHiveMindApiDTO req(m_requestId, GetAgentsListResponseDTO(NULL, 0));
    EXPECT_CALL(m_hostQueueMock, push(testing::_)).WillOnce(testing::Return(false));

    // Then
    bool ret = m_hivemindApiReqHandler->handleMessage(1, 1, req);

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(HiveConnectHiveMindApiMessageHandlerFixture,
       HiveConnectHiveMindApiMessageHandler_handleMessage_agentRequest) {
    // Given
    HiveConnectHiveMindApiDTO req(m_requestId, GetAgentsListRequestDTO());

    // Then
    bool ret = m_hivemindApiReqHandler->handleMessage(1, 1, req);

    // Expect
    EXPECT_FALSE(ret);
}
