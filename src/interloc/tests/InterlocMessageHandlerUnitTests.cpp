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
    CircularQueueInterfaceMock<MessageDTO> m_hostQueueMock;
    CircularQueueInterfaceMock<MessageDTO> m_remoteQueueMock;
    BSPInterfaceMock m_bspMock = BSPInterfaceMock(gc_boardId);

    void SetUp() override {
        m_messageHandler =
            new InterlocMessageHandler(m_loggerMock, m_interlocManagerMock, m_bspMock,
                                       m_inputQueueMock, m_hostQueueMock, m_remoteQueueMock);
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

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_notifyCalibEnded_calledFromHost) {
    MessageDTO messageSent;
    uint16_t initiator = gc_boardId;

    EXPECT_CALL(m_hostQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(true)));
    EXPECT_CALL(m_remoteQueueMock, push(testing::_)).Times(0);
    bool ret = m_messageHandler->notifyCalibrationEnded(initiator);

    EXPECT_TRUE(ret);
    EXPECT_EQ(messageSent.getDestinationId(), initiator);
    EXPECT_EQ(messageSent.getSourceId(), gc_boardId);

    const auto* interlocMessage = std::get_if<InterlocAPIDTO>(&(messageSent.getMessage()));
    const auto* calibMessage = std::get_if<CalibrationMessageDTO>(&(interlocMessage->getAPICall()));

    EXPECT_TRUE(std::holds_alternative<CalibrationEndedDTO>(calibMessage->getCall()));
}

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_notifyCalibEnded_calledFromRemote) {
    MessageDTO messageSent;
    uint16_t initiator = UINT16_MAX;

    EXPECT_CALL(m_remoteQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(true)));
    EXPECT_CALL(m_hostQueueMock, push(testing::_)).Times(0);
    bool ret = m_messageHandler->notifyCalibrationEnded(initiator);

    EXPECT_TRUE(ret);
    EXPECT_EQ(messageSent.getDestinationId(), initiator);
    EXPECT_EQ(messageSent.getSourceId(), gc_boardId);

    const auto* interlocMessage = std::get_if<InterlocAPIDTO>(&(messageSent.getMessage()));
    const auto* calibMessage = std::get_if<CalibrationMessageDTO>(&(interlocMessage->getAPICall()));

    EXPECT_TRUE(std::holds_alternative<CalibrationEndedDTO>(calibMessage->getCall()));
}