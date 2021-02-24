
#include "bittybuzz/BittyBuzzMessageHandler.h"
#include "mocks/BittyBuzzFunctionRegisterInterfaceMock.h"
#include "mocks/CircularQueueInterfaceMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include <gtest/gtest.h>

class BittyBuzzMessageHandlerFixture : public testing::Test {
  protected:
    int m_logCounter;
    std::string m_logLastFormat;
    const uint16_t m_uuid = 42;
    const uint16_t m_srcUuid = 43;

    LoggerInterfaceMock* m_loggerMock;
    BittyBuzzFunctionRegisterInterfaceMock m_functionRegisterMock;
    CircularQueueInterfaceMock<MessageDTO> m_inputQueueMock;
    CircularQueueInterfaceMock<MessageDTO> m_hostOutputQueueMock;
    CircularQueueInterfaceMock<MessageDTO> m_remoteOutputQueueMock;

    BittyBuzzMessageHandler* m_bbzMessageHandler;

    // Messages
    std::string m_functionName = "print";
    FunctionCallRequestDTO* m_fRequest;
    UserCallRequestDTO* m_uRequest;
    RequestDTO* m_request;

    FunctionCallResponseDTO* m_fResponse;
    UserCallResponseDTO* m_uResponse;
    ResponseDTO* m_response;

    void SetUp() override {
        m_fRequest = new FunctionCallRequestDTO(m_functionName.c_str(), NULL, 0);
        m_uRequest =
            new UserCallRequestDTO(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, *m_fRequest);
        m_request = new RequestDTO(1, *m_uRequest);

        m_fResponse = new FunctionCallResponseDTO(GenericResponseStatusDTO::Ok, "");
        m_uResponse =
            new UserCallResponseDTO(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, *m_fResponse);
        m_response = new ResponseDTO(1, *m_uResponse);

        m_loggerMock = new LoggerInterfaceMock(m_logCounter, m_logLastFormat);
        m_bbzMessageHandler = new BittyBuzzMessageHandler(
            m_functionRegisterMock, m_inputQueueMock, m_hostOutputQueueMock,
            m_remoteOutputQueueMock, m_uuid, *m_loggerMock);
    }

    void TearDown() override {
        delete m_bbzMessageHandler;
        delete m_loggerMock;

        delete m_request;
        delete m_uRequest;
        delete m_fRequest;

        delete m_fResponse;
        delete m_uResponse;
        delete m_response;
    }
};

TEST_F(BittyBuzzMessageHandlerFixture, BittyBuzzMessageHandler_processMessage) {
    // Given
    MessageDTO message = MessageDTO(m_srcUuid, m_uuid, *m_request);
    std::optional<std::reference_wrapper<const MessageDTO>> retValue = message;

    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(retValue));
    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_inputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostOutputQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_remoteOutputQueueMock, push(testing::_)).Times(1);

    // Then
    bool ret = m_bbzMessageHandler->processMessage();

    // Expect
    EXPECT_TRUE(ret);
}
