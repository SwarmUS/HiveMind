#include "ApplicationInterface.h"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

class ApplicationInterfaceFixture : public testing::Test {
  protected:
    ApplicationInterface* m_greetHandler;

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
