#include <gtest/gtest.h>
#include <interloc/InterlocMessageHandler.h>
#include <mocks/BSPInterfaceMock.h>
#include <mocks/CircularQueueInterfaceMock.h>
#include <mocks/InterlocManagerInterfaceMock.h>
#include <mocks/LoggerInterfaceMock.h>

class InterlocMessageHandlerFixture : public testing::Test {
  public:
    static constexpr uint16_t gc_boardId = 42;

    InterlocMessageHandler* m_messageHandler;
    InterlocManagerInterfaceMock m_interlocManagerMock;
    LoggerInterfaceMock m_loggerMock;
    CircularQueueInterfaceMock<MessageDTO> m_inputQueueMock;
    BSPInterfaceMock m_bspMock = BSPInterfaceMock(gc_boardId);

    void SetUp() override {
        m_messageHandler = new InterlocMessageHandler(m_loggerMock, m_interlocManagerMock,
                                                      m_bspMock, m_inputQueueMock);
    }

    void TearDown() override { delete m_messageHandler; }
};

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_process_emptyQueue) {
    std::optional<std::reference_wrapper<MessageDTO>> queueValue = {};
    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(queueValue));

    bool ret = m_messageHandler->processMessage();

    EXPECT_TRUE(ret);
}

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_process_popsQueue) {
    auto message =
        MessageDTO(1, gc_boardId, InterlocAPIDTO(CalibrationMessageDTO(StopCalibrationDTO())));
    std::optional<std::reference_wrapper<MessageDTO>> queueValue = message;
    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(queueValue));

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    m_messageHandler->processMessage();
}

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_process_wrongDestination) {
    auto message = MessageDTO(1, 2, InterlocAPIDTO(CalibrationMessageDTO(StopCalibrationDTO())));
    std::optional<std::reference_wrapper<MessageDTO>> queueValue = message;
    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(queueValue));

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    bool ret = m_messageHandler->processMessage();

    EXPECT_FALSE(ret);
}

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_process_wrongType) {
    auto message = MessageDTO(1, gc_boardId, GreetingDTO(3));
    std::optional<std::reference_wrapper<MessageDTO>> queueValue = message;
    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(queueValue));

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    bool ret = m_messageHandler->processMessage();

    EXPECT_FALSE(ret);
}

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_process_startCalib) {
    auto message = MessageDTO(
        1, gc_boardId,
        InterlocAPIDTO(CalibrationMessageDTO(StartCalibrationDTO(CalibrationModeDTO::RESPONDER))));
    std::optional<std::reference_wrapper<MessageDTO>> queueValue = message;
    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(queueValue));

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    bool ret = m_messageHandler->processMessage();

    // TODO: expect stopCalib call

    EXPECT_TRUE(ret);
}

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_process_stopCalib) {
    auto message =
        MessageDTO(1, gc_boardId, InterlocAPIDTO(CalibrationMessageDTO(StopCalibrationDTO())));
    std::optional<std::reference_wrapper<MessageDTO>> queueValue = message;
    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(queueValue));

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    bool ret = m_messageHandler->processMessage();

    // TODO: expect stopCalib call

    EXPECT_TRUE(ret);
}

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_process_setCalibDistance) {
    float distance = 42.42;
    auto message = MessageDTO(
        1, gc_boardId, InterlocAPIDTO(CalibrationMessageDTO(SetCalibrationDistanceDTO(distance))));
    std::optional<std::reference_wrapper<MessageDTO>> queueValue = message;
    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(queueValue));

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    bool ret = m_messageHandler->processMessage();

    // TODO: expect stopCalib call

    EXPECT_TRUE(ret);
}

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_process_unknownCalibMessage) {
    CalibrationMessage calibMsg{};
    calibMsg.which_call = PB_SIZE_MAX;
    auto message = MessageDTO(1, gc_boardId, InterlocAPIDTO(CalibrationMessageDTO(calibMsg)));
    std::optional<std::reference_wrapper<MessageDTO>> queueValue = message;
    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(queueValue));

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    bool ret = m_messageHandler->processMessage();

    EXPECT_FALSE(ret);
}