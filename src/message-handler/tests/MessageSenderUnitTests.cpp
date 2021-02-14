#include "mocks/CircularQueueInterfaceMock.h"
#include "mocks/HiveMindHostSerializerInterfaceMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include <message-handler/MessageSender.h>

class MessageSenderFixture : public testing::Test {
  protected:
    MessageSender* m_messageSender;

    CircularQueueInterfaceMack<MessageDTO> m_inputQueueMock;
    HiveMindHostSerializerInterfaceMock m_serializerMock;
    LoggerInterfaceMock m_loggerMock;
    MessageDTO m_message;

    void SetUp() override {
        m_messageSender = new MessageSender(m_inputQueueMock, m_serializerMock, m_loggerMock);
    }
    void TearDown() override { delete m_messageSender; }
};

TEST_F(MessageSenderFixture, MessageSender_processAndSerialize_validMessage) {
    // Given
    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(m_message));
    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_serializerMock, serializeToStream(testing::_))
        .Times(1)
        .WillOnce(testing::Return(true));

    // Then
    bool ret = m_messageSender->processAndSerialize();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageSenderFixture, MessageSender_processAndSerialize_emptyMessage) {
    // Given
    const std::optional<std::reference_wrapper<const MessageDTO>> emptyMessage = {};
    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(emptyMessage));
    EXPECT_CALL(m_inputQueueMock, pop).Times(0);
    EXPECT_CALL(m_serializerMock, serializeToStream(testing::_)).Times(0);

    // Then
    bool ret = m_messageSender->processAndSerialize();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(MessageSenderFixture, MessageSender_processAndSerialize_invalidDeserialization) {
    // Given
    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(m_message));
    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_serializerMock, serializeToStream(testing::_))
        .Times(1)
        .WillOnce(testing::Return(false));

    // Then
    bool ret = m_messageSender->processAndSerialize();

    // Expect
    EXPECT_FALSE(ret);
}
