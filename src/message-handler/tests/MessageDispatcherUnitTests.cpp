#include "mocks/CircularQueueInterfaceMock.h"
#include "mocks/HiveMindHostDeserializerInterfaceMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include <message-handler/MessageDispatcher.h>

class MessageDispatcherFixture : public testing::Test {
  protected:
    MessageDispatcher* m_messageDispatcher;

    CircularQueueInterfaceMack<MessageDTO> m_buzzQueue;
    CircularQueueInterfaceMack<MessageDTO> m_hostQueue;
    CircularQueueInterfaceMack<MessageDTO> m_remoteQueue;
    HiveMindHostDeserializerInterfaceMock m_deserializerMock;
    LoggerInterfaceMock m_loggerMock;
    MessageDTO m_message;

    FunctionCallRequestDTO* m_fRequest;
    UserCallRequestDTO* m_uReqDTO;
    RequestDTO* m_reqDTO;

    FunctionCallResponseDTO* m_fResponse;
    UserCallResponseDTO* m_uResponse;
    ResponseDTO* m_response;

    void SetUp() override {
        m_fRequest = new FunctionCallRequestDTO(NULL, NULL, 0);
        m_uReqDTO = new UserCallRequestDTO(UserCallDestinationDTO::HOST, *m_fRequest);
        m_reqDTO = new RequestDTO(1, *m_uReqDTO);

        m_fResponse = new FunctionCallResponseDTO(GenericResponseStatusDTO::Ok, "");
        m_uResponse = new UserCallResponseDTO(UserCallDestinationDTO::BUZZ, *m_fResponse);
        m_response = new ResponseDTO(1, *m_uResponse);

        m_message = MessageDTO(1, 1, *m_reqDTO);
        m_messageDispatcher = new MessageDispatcher(m_buzzQueue, m_hostQueue, m_remoteQueue,
                                                    m_deserializerMock, m_loggerMock);
    }
    void TearDown() override {
        delete m_messageDispatcher;
        delete m_reqDTO;
        delete m_uReqDTO;
        delete m_fRequest;

        delete m_fResponse;
        delete m_uResponse;
        delete m_response;
    }
};

TEST_F(MessageDispatcher, MessageSender_processAndSerialize_validMessage) {
    // Given
    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(m_message));
    EXPECT_CALL(m_serializerMock, serializeToStream(testing::_))
        .Times(1)
        .WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageSender->processAndSerialize();

    // Expect
    EXPECT_TRUE(ret);
}
