#include "mocks/CircularQueueInterfaceMock.h"
#include "mocks/LoggerInterfaceMock.h"
#include <bittybuzz/BittyBuzzMessageService.h>
#include <gtest/gtest.h>

class BittyBuzzMessageServiceTestFixture : public testing::Test {
  protected:
    BittyBuzzMessageService* m_messageService;

    CircularQueueInterfaceMock<MessageDTO> m_hostOutputQueueMock;
    CircularQueueInterfaceMock<MessageDTO> m_remoteOutputQueueMock;
    LoggerInterfaceMock* m_loggerMock;
    int m_logCounter = 0;
    const uint16_t m_bspUUID = 42;
    std::string m_logLastFormat;

    void SetUp() override {
        m_loggerMock = new LoggerInterfaceMock(m_logCounter, m_logLastFormat);
        m_messageService = new BittyBuzzMessageService(
            m_hostOutputQueueMock, m_remoteOutputQueueMock, m_bspUUID, *m_loggerMock);
    }

    void TearDown() override {
        delete m_loggerMock;
        delete m_messageService;
    }
};

TEST_F(BittyBuzzMessageServiceTestFixture, BittyBuzzMessageService_callFunction) {
    // Given

    // Then
    m_messageService->callFunction(uint16_t id, const char* functionName,
                                   const FunctionCallArgumentDTO* args, uint16_t argsLength);

    // Expect
    EXPECT_EQ(m_logCounter, 0);
    EXPECT_STREQ(ret.value(), m_array[1].second);
    EXPECT_EQ(ret.operator bool(), true);
}
