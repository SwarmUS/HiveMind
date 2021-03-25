#include "mocks/BSPInterfaceMock.h"
#include "mocks/CircularQueueInterfaceMock.h"
#include "mocks/HiveMindHostDeserializerInterfaceMock.h"
#include "mocks/HiveMindHostSerializerInterfaceMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include "utils/DTOMatchers.h"
#include <message-handler/GreetHandler.h>

class GreetHandlerFixture : public testing::Test {
  protected:
    GreetHandler* m_greetHandler;

    BSPInterfaceMock* m_bspMock;
    HiveMindHostSerializerInterfaceMock m_serializerMock;
    HiveMindHostDeserializerInterfaceMock m_deserializerMock;
    LoggerInterfaceMock m_loggerMock;
    MessageDTO m_message;

    uint16_t m_uuid = 42;
    void SetUp() override {
        m_bspMock = new BSPInterfaceMock(m_uuid);
        m_greetHandler = new GreetHandler(m_serializerMock, m_deserializerMock, *m_bspMock);
    }
    void TearDown() override { delete m_greetHandler; }
};

TEST_F(GreetHandlerFixture, GreetHandler_greet_valid) {
    // Given
    m_message.setMessage(GreetingDTO(42));
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_serializerMock, serializeToStream(MessageGreetingDTOMatcher(m_uuid)))
        .WillOnce(testing::Return(true));

    // Then
    bool ret = m_greetHandler->greet();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(GreetHandlerFixture, GreetHandler_greet_invalidSerialization) {
    // Given
    m_message.setMessage(GreetingDTO(42));
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_serializerMock, serializeToStream(MessageGreetingDTOMatcher(m_uuid)))
        .WillOnce(testing::Return(false));

    // Then
    bool ret = m_greetHandler->greet();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(GreetHandlerFixture, GreetHandler_greet_invalidDeserialization) {
    // Given
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::Return(false));
    EXPECT_CALL(m_serializerMock, serializeToStream(testing::_)).Times(0);

    // Then
    bool ret = m_greetHandler->greet();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(GreetHandlerFixture, GreetHandler_greet_notGreet) {
    // Given
    m_message.setMessage(std::monostate());
    EXPECT_CALL(m_deserializerMock, deserializeFromStream(testing::_))
        .WillOnce(testing::DoAll(testing::SetArgReferee<0>(m_message), testing::Return(true)));
    EXPECT_CALL(m_serializerMock, serializeToStream(testing::_)).Times(0);

    // Then
    bool ret = m_greetHandler->greet();

    // Expect
    EXPECT_FALSE(ret);
}

TEST_F(GreetHandlerFixture, GreetHandler_sendGreet_valid) {
    // Given
    EXPECT_CALL(m_serializerMock, serializeToStream(MessageGreetingDTOMatcher(m_uuid)))
        .WillOnce(testing::Return(true));

    // Then
    bool ret = m_greetHandler->sendGreet();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(GreetHandlerFixture, GreetHandler_sendGreet_invalidSerialization) {
    // Given
    EXPECT_CALL(m_serializerMock, serializeToStream(testing::_)).WillOnce(testing::Return(false));

    // Then
    bool ret = m_greetHandler->sendGreet();

    // Expect
    EXPECT_FALSE(ret);
}
