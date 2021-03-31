#include "mocks/BSPInterfaceMock.h"
#include "mocks/CircularQueueInterfaceMock.h"
#include "mocks/HiveMindHostDeserializerInterfaceMock.h"
#include "mocks/HiveMindHostSerializerInterfaceMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include "utils/DTOMatchers.h"
#include <message-handler/GreetSender.h>

class GreetSenderFixture : public testing::Test {
  protected:
    GreetSender* m_greetHandler;

    BSPInterfaceMock* m_bspMock;
    CircularQueueInterfaceMock<MessageDTO> m_outputQueueMock;
    LoggerInterfaceMock m_loggerMock;
    MessageDTO m_message;

    uint16_t m_uuid = 42;
    void SetUp() override {
        m_bspMock = new BSPInterfaceMock(m_uuid);
        m_greetHandler = new GreetSender(m_outputQueueMock, *m_bspMock);
    }
    void TearDown() override { delete m_greetHandler; }
};

TEST_F(GreetSenderFixture, GreetSender_sendGreet_valid) {
    // Given
    EXPECT_CALL(m_outputQueueMock, push(MessageGreetingDTOMatcher(m_uuid)))
        .WillOnce(testing::Return(true));

    // Then
    bool ret = m_greetHandler->sendGreet();

    // Expect
    EXPECT_TRUE(ret);
}

TEST_F(GreetSenderFixture, GreetSender_sendGreet_invalidSerialization) {
    // Given
    EXPECT_CALL(m_outputQueueMock, push(testing::_)).WillOnce(testing::Return(false));

    // Then
    bool ret = m_greetHandler->sendGreet();

    // Expect
    EXPECT_FALSE(ret);
}
