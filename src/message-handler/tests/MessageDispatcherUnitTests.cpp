#include "mocks/BSPInterfaceMock.h"
#include "mocks/CircularQueueInterfaceMock.h"
#include "mocks/GreetSenderInterfaceMock.h"
#include "mocks/HiveConnectHiveMindApiMessageHandlerInterfaceMock.h"
#include "mocks/HiveMindHostApiRequestHandlerInterfaceMock.h"
#include "mocks/HiveMindHostDeserializerInterfaceMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include <message-handler/MessageDispatcher.h>

class MessageDispatcherFixture : public testing::Test {
  protected:
    MessageDispatcher* m_messageDispatcher;

    testing::StrictMock<CircularQueueInterfaceMock<MessageDTO>> m_buzzQueue;
    testing::StrictMock<CircularQueueInterfaceMock<MessageDTO>> m_hostQueue;
    testing::StrictMock<CircularQueueInterfaceMock<MessageDTO>> m_remoteQueue;
    testing::StrictMock<CircularQueueInterfaceMock<MessageDTO>> m_interlocQueue;
    testing::StrictMock<HiveMindHostDeserializerInterfaceMock> m_deserializerMock;
    testing::StrictMock<HiveMindHostApiRequestHandlerInterfaceMock> m_hivemindApiReqHandlerMock;
    testing::StrictMock<HiveConnectHiveMindApiMessageHandlerInterfaceMock>
        m_hiveconnectHivemindHandler;
    testing::StrictMock<GreetSenderInterfaceMock> m_greetSenderMock;
    BSPInterfaceMock* m_bspMock;
    LoggerInterfaceMock m_loggerMock;
    MessageDTO m_message;

    FunctionCallRequestDTO* m_fRequest;
    UserCallRequestDTO* m_uRequest;
    RequestDTO* m_request;

    FunctionCallResponseDTO* m_fResponse;
    UserCallResponseDTO* m_uResponse;
    ResponseDTO* m_response;

    const uint16_t m_uuid = 1;
    const uint16_t m_srcUuid = 42;

    void SetUp() override {
        m_bspMock = new BSPInterfaceMock(m_uuid);
        m_fRequest = new FunctionCallRequestDTO(NULL, NULL, 0);
        m_uRequest =
            new UserCallRequestDTO(UserCallTargetDTO::HOST, UserCallTargetDTO::BUZZ, *m_fRequest);
        m_request = new RequestDTO(1, *m_uRequest);

        m_fResponse = new FunctionCallResponseDTO(GenericResponseStatusDTO::Ok, "");
        m_uResponse =
            new UserCallResponseDTO(UserCallTargetDTO::HOST, UserCallTargetDTO::BUZZ, *m_fResponse);
        m_response = new ResponseDTO(1, *m_uResponse);

        m_messageDispatcher = new MessageDispatcher(
            m_buzzQueue, m_hostQueue, m_remoteQueue, m_interlocQueue, m_deserializerMock,
            m_hivemindApiReqHandlerMock, m_hiveconnectHivemindHandler, m_greetSenderMock,
            *m_bspMock, m_loggerMock);
    }
    void TearDown() override {
        delete m_bspMock;
        delete m_messageDispatcher;
        delete m_request;
        delete m_uRequest;
        delete m_fRequest;

        delete m_fResponse;
        delete m_uResponse;
        delete m_response;
    }
};

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_validUserCallRequest_buzz) {
    // Given
    m_message = MessageDTO(m_srcUuid, m_uuid, *m_request);
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_buzzQueue, push(testing::_)).Times(1).WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_invalidUserCallRequest_buzz) {
    // Given
    m_message = MessageDTO(m_srcUuid, m_uuid, *m_request);
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(false)));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_validUserCallRequest_host) {
    // Given
    UserCallRequestDTO uRequest(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, *m_fRequest);
    RequestDTO request(1, uRequest);
    m_message = MessageDTO(m_srcUuid, m_uuid, request);

    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_hostQueue, push(testing::_)).Times(1).WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_UnknownSource_validUserCall) {
    // Given
    UserCallRequestDTO uRequest(UserCallTargetDTO::UNKNOWN, UserCallTargetDTO::HOST, *m_fRequest);
    RequestDTO request(1, uRequest);
    m_message = MessageDTO(m_srcUuid, m_uuid, request);

    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_hostQueue, push(testing::_)).Times(1).WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_invalidUserCallRequest_host) {
    // Given
    UserCallRequestDTO uRequest(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, *m_fRequest);
    RequestDTO request(1, uRequest);
    m_message = MessageDTO(m_srcUuid, m_uuid, request);

    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(false)));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_validUserCallRequest_Unknown_returnfalse) {
    // Given
    UserCallRequestDTO uRequest(UserCallTargetDTO::HOST, UserCallTargetDTO::UNKNOWN, *m_fRequest);
    RequestDTO request(1, uRequest);
    m_message = MessageDTO(m_srcUuid, m_uuid, request);

    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_validUserCallRequest_remote) {
    // Given
    m_message = MessageDTO(m_uuid, m_srcUuid, *m_request);
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_remoteQueue, push(testing::_)).Times(1).WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_invalidUserCallRequest_remote) {
    // Given
    m_message = MessageDTO(m_uuid, m_srcUuid, *m_request);
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(false)));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_validUserCallResponse_buzz) {
    // Given
    m_message = MessageDTO(m_srcUuid, m_uuid, *m_response);
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_buzzQueue, push(testing::_)).Times(1).WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_UserCallResponse_buzz_QueueFull) {
    // Given
    m_message = MessageDTO(m_srcUuid, m_uuid, *m_response);
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_buzzQueue, push(testing::_)).Times(1).WillOnce(testing::Return(false));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_invalidUserCallResponse_buzz) {
    // Given
    m_message = MessageDTO(m_srcUuid, m_uuid, *m_response);
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(false)));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_validUserCallResponse_host) {
    // Given
    UserCallResponseDTO uResponse(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, *m_fResponse);
    ResponseDTO response(1, uResponse);
    m_message = MessageDTO(m_srcUuid, m_uuid, response);

    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_hostQueue, push(testing::_)).Times(1).WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_UserCallResponse_host_QueueFUll) {
    // Given
    UserCallResponseDTO uResponse(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, *m_fResponse);
    ResponseDTO response(1, uResponse);
    m_message = MessageDTO(m_srcUuid, m_uuid, response);

    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_hostQueue, push(testing::_)).Times(1).WillOnce(testing::Return(false));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(MessageDispatcherFixture, MessageDispatcher_deserializeAndDispatch_unknownSource_valid) {
    // Given
    UserCallResponseDTO uResponse(UserCallTargetDTO::UNKNOWN, UserCallTargetDTO::HOST,
                                  *m_fResponse);
    ResponseDTO response(1, uResponse);
    m_message = MessageDTO(m_srcUuid, m_uuid, response);

    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_hostQueue, push(testing::_)).Times(1).WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_invalidUserCallResponse_host) {
    // Given
    UserCallResponseDTO uResponse(UserCallTargetDTO::BUZZ, UserCallTargetDTO::HOST, *m_fResponse);
    ResponseDTO response(1, uResponse);
    m_message = MessageDTO(m_srcUuid, m_uuid, response);

    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(false)));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(
    MessageDispatcherFixture,
    MessageDispatcher_deserializeAndDispatch_validUserCallResponse_UnknownDestination_returnfalse) {
    // Given
    UserCallResponseDTO uResponse(UserCallTargetDTO::HOST, UserCallTargetDTO::UNKNOWN,
                                  *m_fResponse);
    ResponseDTO response(1, uResponse);
    m_message = MessageDTO(m_srcUuid, m_uuid, response);

    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_validUserCallResponse_remote) {
    // Given
    m_message = MessageDTO(m_uuid, m_srcUuid, *m_response);
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_remoteQueue, push(testing::_)).Times(1).WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_UserCallResponse_noMoreSpaceInQueue) {
    // Given
    m_message = MessageDTO(m_uuid, m_srcUuid, *m_response);
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_remoteQueue, push(testing::_)).Times(1).WillOnce(testing::Return(false));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_invalidUserCallResponse_remote) {
    // Given
    m_message = MessageDTO(m_uuid, m_srcUuid, *m_response);
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(false)));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_validGenericResponse_sendToHost) {
    // Given
    m_response->setResponse(GenericResponseDTO(GenericResponseStatusDTO::Ok, ""));
    m_message = MessageDTO(m_srcUuid, m_uuid, *m_response);
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_hostQueue, push(testing::_)).Times(1).WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_validGenericResponse_sendToRemote) {
    // Given
    m_response->setResponse(GenericResponseDTO(GenericResponseStatusDTO::Ok, ""));
    m_message = MessageDTO(m_uuid, m_srcUuid, *m_response);
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_remoteQueue, push(testing::_)).Times(1).WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_Generic_noMoreSpaceInQueue) {
    // Given
    m_response->setResponse(GenericResponseDTO(GenericResponseStatusDTO::Ok, ""));
    m_message = MessageDTO(m_uuid, m_srcUuid, *m_response);
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_remoteQueue, push(testing::_)).Times(1).WillOnce(testing::Return(false));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_HiveMindHostApiRequest_handlerCalled_valid) {
    // Given
    m_request->setRequest(HiveMindHostApiRequestDTO(BytesDTO(0, 0, true, NULL, 0)));
    m_message = MessageDTO(m_uuid, m_uuid, *m_request);
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_hivemindApiReqHandlerMock, handleRequest).WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_HiveMindHostApiRequest_handlerCalled_invalid) {
    // Given
    m_request->setRequest(HiveMindHostApiRequestDTO(BytesDTO(0, 0, true, NULL, 0)));
    m_message = MessageDTO(m_uuid, m_uuid, *m_request);
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_hivemindApiReqHandlerMock, handleRequest).WillOnce(testing::Return(false));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(MessageDispatcherFixture, MessageDispatcher_deserializeAndDispatch_BuzzMessage_valid) {
    // Given
    BuzzMessageDTO bbzMessage(NULL, 0);
    BuzzMessagesDTO bbzMessages(&bbzMessage, 1);
    VmMessageDTO vmMessage(bbzMessages);
    m_message = MessageDTO(m_srcUuid, m_uuid, vmMessage);
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_buzzQueue, push(testing::_)).Times(1).WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageDispatcherFixture, MessageDispatcher_deserializeAndDispatch_BuzzMessage_invalid) {
    // Given
    BuzzMessageDTO bbzMessage(NULL, 0);
    BuzzMessagesDTO bbzMessages(&bbzMessage, 1);
    VmMessageDTO vmMessage(bbzMessages);
    m_message = MessageDTO(m_srcUuid, m_uuid, vmMessage);
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_buzzQueue, push(testing::_)).Times(1).WillOnce(testing::Return(false));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(MessageDispatcherFixture, MessageDispatcher_deserializeAndDispatch_greet_valid) {
    // Given
    m_message = MessageDTO(m_srcUuid, m_uuid, GreetingDTO(0));
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_greetSenderMock, sendGreet).WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageDispatcherFixture, MessageDispatcher_deserializeAndDispatch_greet_invalid) {
    // Given
    m_message = MessageDTO(m_srcUuid, m_uuid, GreetingDTO(0));
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_greetSenderMock, sendGreet).WillOnce(testing::Return(false));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(MessageDispatcherFixture, MessageDispatcher_deserializeAndDispatch_greet_remote_valid) {
    // Given
    m_message = MessageDTO(m_uuid, m_srcUuid, GreetingDTO(42));
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_greetSenderMock, sendGreet).WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageDispatcherFixture, MessageDispatcher_deserializeAndDispatch_validInterlocAPIMessage) {
    // Given
    m_message = MessageDTO(m_srcUuid, m_uuid,
                           InterlocAPIDTO(SetInterlocStateDTO(InterlocStateDTO::STANDBY)));
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_interlocQueue, push(testing::_)).Times(1).WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_invalidInterlocAPIMessage) {
    // Given
    m_message = MessageDTO(m_srcUuid, m_uuid,
                           InterlocAPIDTO(SetInterlocStateDTO(InterlocStateDTO::STANDBY)));
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_interlocQueue, push(testing::_)).Times(1).WillOnce(testing::Return(false));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_HiveConnectHiveMindApi_valid) {
    // Given
    m_message =
        MessageDTO(m_srcUuid, m_uuid, HiveConnectHiveMindApiDTO(24, GetAgentsListRequestDTO()));
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_hiveconnectHivemindHandler, handleMessage).WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_invalidInterlocAPIMessage_invalid) {
    // Given
    m_message =
        MessageDTO(m_srcUuid, m_uuid, HiveConnectHiveMindApiDTO(24, GetAgentsListRequestDTO()));
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_hiveconnectHivemindHandler, handleMessage).WillOnce(testing::Return(false));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_broadcastMessage_hostAndRemote) {
    // Given
    m_message = MessageDTO(m_srcUuid, 0, *m_request);
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_buzzQueue, push(testing::_)).WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageDispatcherFixture,
       MessageDispatcher_deserializeAndDispatch_broadcastMessage_remoteOnly) {
    // Given
    m_message = MessageDTO(m_uuid, 0, *m_request);
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_buzzQueue, push(testing::_)).WillOnce(testing::Return(true));
    EXPECT_CALL(m_remoteQueue, push(testing::_)).WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageDispatcher->deserializeAndDispatch();

    // Expect
    EXPECT_TRUE(ret);
}
