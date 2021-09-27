#include <gtest/gtest.h>
#include <interloc/InterlocMessageHandler.h>
#include <mocks/BSPInterfaceMock.h>
#include <mocks/CircularQueueInterfaceMock.h>
#include <mocks/InterlocManagerInterfaceMock.h>
#include <mocks/LoggerInterfaceMock.h>
#include <pheromones/MessageDTO.h>

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

    // Sends a state change message and has the MessageHandler read the message and call the
    // interloc manager
    void setInterlocState(uint32_t msgSourceId, InterlocStateDTO state) {
        auto message =
            MessageDTO(msgSourceId, gc_boardId, InterlocAPIDTO(SetInterlocStateDTO(state)));
        std::optional<std::reference_wrapper<MessageDTO>> queueValue = message;
        EXPECT_CALL(m_inputQueueMock, peek).WillRepeatedly(testing::Return(queueValue));

        EXPECT_CALL(m_inputQueueMock, pop);
        EXPECT_CALL(m_interlocManagerMock, setInterlocManagerState);
        m_messageHandler->processMessage();
    }

    static BspInterlocRawAngleData fillAngleData(uint32_t numFrames, uint32_t numBeeboards) {
        BspInterlocRawAngleData rawAngleData;

        rawAngleData.m_framesLength = numFrames;
        for (unsigned int i = 0; i < numFrames; i++) {
            rawAngleData.m_frames[i].m_frameInfosLength = numBeeboards;

            for (unsigned int j = 0; j < numBeeboards; j++) {
                rawAngleData.m_frames[i].m_frameInfos[j].m_beeboardPort = j;
                rawAngleData.m_frames[i].m_frameInfos[j].m_accumulatorAngle = i + j + 1;
                rawAngleData.m_frames[i].m_frameInfos[j].m_sfdAngle = i + j + 2;
                rawAngleData.m_frames[i].m_frameInfos[j].m_rxTimestamp = i + j + 3;
                rawAngleData.m_frames[i].m_frameInfos[j].m_messageId = i + j + 4;
            }
        }

        return rawAngleData;
    }

    static void checkAngleDTO(uint32_t numFrames,
                              uint8_t numBeeboards,
                              uint32_t startingFrameId,
                              const BspInterlocFrameAngleRawData* frames,
                              const InterlocRawAngleDataDTO* dto) {
        for (unsigned int i = 0; i < numFrames; i++) {
            EXPECT_EQ(dto->getFrames()[i].getFrameInfosLength(), numBeeboards);
            EXPECT_EQ(dto->getFrames()[i].getFrameId(), startingFrameId + i);

            for (unsigned int j = 0; j < numBeeboards; j++) {
                EXPECT_EQ(dto->getFrames()[i].getFrameInfos()[j].getBeeboardPort(),
                          frames[i].m_frameInfos[j].m_beeboardPort);
                EXPECT_EQ(dto->getFrames()[i].getFrameInfos()[j].getAccumulatorAngle(),
                          frames[i].m_frameInfos[j].m_accumulatorAngle);
                EXPECT_EQ(dto->getFrames()[i].getFrameInfos()[j].getRxTimestamp(),
                          frames[i].m_frameInfos[j].m_rxTimestamp);
                EXPECT_EQ(dto->getFrames()[i].getFrameInfos()[j].getSfdAngle(),
                          frames[i].m_frameInfos[j].m_sfdAngle);
                EXPECT_EQ(dto->getFrames()[i].getFrameInfos()[j].getMessageId(),
                          frames[i].m_frameInfos[j].m_messageId);
            }
        }
    }
};

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_process_emptyQueue) {
    std::optional<std::reference_wrapper<MessageDTO>> queueValue = {};
    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(queueValue));

    bool ret = m_messageHandler->processMessage();

    EXPECT_TRUE(ret);
}

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_process_popsQueue) {
    auto message =
        MessageDTO(1, gc_boardId, InterlocAPIDTO(SetInterlocStateDTO(InterlocStateDTO::STANDBY)));
    std::optional<std::reference_wrapper<MessageDTO>> queueValue = message;
    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(queueValue));

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    m_messageHandler->processMessage();
}

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_process_wrongDestination) {
    auto message = MessageDTO(
        1, 2, InterlocAPIDTO(InterlocAPIDTO(SetInterlocStateDTO(InterlocStateDTO::STANDBY))));
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

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_process_configureTWRDistance) {
    float distance = 42.42;
    uint16_t callArg;
    auto message =
        MessageDTO(1, gc_boardId,
                   InterlocAPIDTO(InterlocConfigurationDTO(ConfigureTWRCalibrationDTO(distance))));
    std::optional<std::reference_wrapper<MessageDTO>> queueValue = message;
    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(queueValue));

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_interlocManagerMock, configureTWRCalibration(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&callArg)));
    bool ret = m_messageHandler->processMessage();

    EXPECT_TRUE(ret);
    EXPECT_EQ(callArg, (uint16_t)(distance * 100));
}

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_process_configureAngleCalib) {
    uint32_t numberOfFrames = 42;
    uint32_t callArg;
    auto message = MessageDTO(
        1, gc_boardId,
        InterlocAPIDTO(InterlocConfigurationDTO(ConfigureAngleCalibrationDTO(numberOfFrames))));
    std::optional<std::reference_wrapper<MessageDTO>> queueValue = message;
    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(queueValue));

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_interlocManagerMock, configureAngleCalibration(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&callArg)));
    bool ret = m_messageHandler->processMessage();

    EXPECT_TRUE(ret);
    EXPECT_EQ(callArg, numberOfFrames);
}

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_process_setState) {
    InterlocStateDTO state = InterlocStateDTO::STANDBY;
    InterlocStateDTO callArg;
    auto message = MessageDTO(1, gc_boardId, InterlocAPIDTO(SetInterlocStateDTO(state)));
    std::optional<std::reference_wrapper<MessageDTO>> queueValue = message;
    EXPECT_CALL(m_inputQueueMock, peek).Times(1).WillOnce(testing::Return(queueValue));

    EXPECT_CALL(m_inputQueueMock, pop).Times(1);
    EXPECT_CALL(m_interlocManagerMock, setInterlocManagerState(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&callArg)));

    bool ret = m_messageHandler->processMessage();

    EXPECT_TRUE(ret);
    EXPECT_EQ(callArg, state);
}

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_stateChangeCallback_messageToHost) {
    uint16_t sourceId = gc_boardId;

    InterlocStateDTO fromState = InterlocStateDTO::STANDBY;
    InterlocStateDTO toState = InterlocStateDTO::OPERATING;
    MessageDTO messageSent;

    // Send message so the manager knows who to respond to
    setInterlocState(sourceId, toState);

    EXPECT_CALL(m_hostQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(true)));
    EXPECT_CALL(m_remoteQueueMock, push(testing::_)).Times(0);

    m_interlocManagerMock.m_stateChangeCallback(m_interlocManagerMock.m_stateChangeContext,
                                                fromState, toState);

    EXPECT_EQ(messageSent.getDestinationId(), sourceId);
    EXPECT_EQ(messageSent.getSourceId(), gc_boardId);

    const auto* interlocMessage = std::get_if<InterlocAPIDTO>(&(messageSent.getMessage()));
    const auto* outputMessage =
        std::get_if<InterlocOutputMessageDTO>(&(interlocMessage->getAPICall()));
    const auto* stateChangeMessage =
        std::get_if<InterlocStateChangeDTO>(&(outputMessage->getMessage()));

    EXPECT_EQ(stateChangeMessage->getPreviousState(), fromState);
    EXPECT_EQ(stateChangeMessage->getNewState(), toState);
}

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_stateChangeCallback_messageToRemote) {
    uint16_t sourceId = 10;

    InterlocStateDTO fromState = InterlocStateDTO::STANDBY;
    InterlocStateDTO toState = InterlocStateDTO::OPERATING;
    MessageDTO messageSent;

    // Send message so the manager knows who to respond to
    setInterlocState(sourceId, toState);

    EXPECT_CALL(m_remoteQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(true)));
    EXPECT_CALL(m_hostQueueMock, push(testing::_)).Times(0);

    m_interlocManagerMock.m_stateChangeCallback(m_interlocManagerMock.m_stateChangeContext,
                                                fromState, toState);

    EXPECT_EQ(messageSent.getDestinationId(), sourceId);
    EXPECT_EQ(messageSent.getSourceId(), gc_boardId);

    const auto* interlocMessage = std::get_if<InterlocAPIDTO>(&(messageSent.getMessage()));
    const auto* outputMessage =
        std::get_if<InterlocOutputMessageDTO>(&(interlocMessage->getAPICall()));
    const auto* stateChangeMessage =
        std::get_if<InterlocStateChangeDTO>(&(outputMessage->getMessage()));

    EXPECT_EQ(stateChangeMessage->getPreviousState(), fromState);
    EXPECT_EQ(stateChangeMessage->getNewState(), toState);
}

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_stateChangeCallback_messageToUnknown) {
    InterlocStateDTO fromState = InterlocStateDTO::STANDBY;
    InterlocStateDTO toState = InterlocStateDTO::OPERATING;

    EXPECT_CALL(m_remoteQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostQueueMock, push(testing::_)).Times(0);

    m_interlocManagerMock.m_stateChangeCallback(m_interlocManagerMock.m_stateChangeContext,
                                                fromState, toState);
}

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_angleDataCallback_onePacket) {
    uint16_t sourceId = gc_boardId;
    MessageDTO messageSent;

    uint32_t numFrames = InterlocRawAngleDataDTO::INTERLOC_RAW_ANGLE_FRAMES_MAX_SIZE;
    uint32_t numBeeboards = InterlocRxFrameRawAngleDataDTO::INTERLOC_BEEBOARDS_SIZE_MAX_LENGTH;

    BspInterlocRawAngleData rawAngleData = fillAngleData(numFrames, numBeeboards);

    // Send message so the manager knows who to respond to
    setInterlocState(sourceId, InterlocStateDTO::ANGLE_CALIB_RECEIVER);

    // Mock the call that the InterlocManager would do once it has data
    EXPECT_CALL(m_remoteQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostQueueMock, isFull).Times(1).WillOnce(testing::Return(false));
    EXPECT_CALL(m_hostQueueMock, push(testing::_))
        .Times(1)
        .WillOnce(testing::DoAll(testing::SaveArg<0>(&messageSent), testing::Return(true)));

    m_interlocManagerMock.m_angleDataCallback(m_interlocManagerMock.m_angleDataContext,
                                              rawAngleData);

    EXPECT_EQ(messageSent.getDestinationId(), sourceId);
    EXPECT_EQ(messageSent.getSourceId(), gc_boardId);

    const auto* interlocMessage = std::get_if<InterlocAPIDTO>(&(messageSent.getMessage()));
    const auto* outputMessage =
        std::get_if<InterlocOutputMessageDTO>(&(interlocMessage->getAPICall()));
    const auto* angleDataDTO = std::get_if<InterlocRawAngleDataDTO>(&(outputMessage->getMessage()));

    EXPECT_EQ(angleDataDTO->getFramesLength(), numFrames);
    checkAngleDTO(numFrames, numBeeboards, 0, rawAngleData.m_frames.data(), angleDataDTO);
}

TEST_F(InterlocMessageHandlerFixture, InterlocMessageHandler_angleDataCallback_multiplePackets) {
    uint16_t sourceId = gc_boardId;

    constexpr uint8_t numFullPackets = 3;
    constexpr uint8_t numFramesOverflow = 2;
    constexpr uint8_t numPackets = numFullPackets + 1;
    constexpr uint8_t maxFrameLength = InterlocRawAngleDataDTO::INTERLOC_RAW_ANGLE_FRAMES_MAX_SIZE;
    uint32_t numFrames = maxFrameLength * numFullPackets + numFramesOverflow;
    uint32_t numBeeboards = InterlocRxFrameRawAngleDataDTO::INTERLOC_BEEBOARDS_SIZE_MAX_LENGTH;

    BspInterlocRawAngleData rawAngleData = fillAngleData(numFrames, numBeeboards);
    std::vector<MessageDTO> sentMessages;

    // Send message so the manager knows who to respond to
    setInterlocState(sourceId, InterlocStateDTO::ANGLE_CALIB_RECEIVER);

    // Mock the call that the InterlocManager would do once it has data
    EXPECT_CALL(m_remoteQueueMock, push(testing::_)).Times(0);
    EXPECT_CALL(m_hostQueueMock, isFull).WillRepeatedly(testing::Return(false));
    EXPECT_CALL(m_hostQueueMock, push(testing::_))
        .Times(numPackets)
        .WillRepeatedly(testing::DoAll(
            testing::Invoke([&sentMessages](MessageDTO argMsg) { sentMessages.push_back(argMsg); }),
            testing::Return(true)));

    m_interlocManagerMock.m_angleDataCallback(m_interlocManagerMock.m_angleDataContext,
                                              rawAngleData);

    for (unsigned int i = 0; i < numPackets; i++) {
        EXPECT_EQ(sentMessages[i].getDestinationId(), sourceId);
        EXPECT_EQ(sentMessages[i].getSourceId(), gc_boardId);

        const auto* interlocMessage = std::get_if<InterlocAPIDTO>(&(sentMessages[i].getMessage()));
        const auto* outputMessage =
            std::get_if<InterlocOutputMessageDTO>(&(interlocMessage->getAPICall()));
        const auto* angleDataDTO =
            std::get_if<InterlocRawAngleDataDTO>(&(outputMessage->getMessage()));

        if (i < numFullPackets) {
            EXPECT_EQ(angleDataDTO->getFramesLength(),
                      InterlocRawAngleDataDTO::INTERLOC_RAW_ANGLE_FRAMES_MAX_SIZE);
            checkAngleDTO(maxFrameLength, numBeeboards, i * maxFrameLength,
                          &rawAngleData.m_frames[i * maxFrameLength], angleDataDTO);
        } else {
            EXPECT_EQ(angleDataDTO->getFramesLength(), numFramesOverflow);
            checkAngleDTO(numFramesOverflow, numBeeboards, i * maxFrameLength,
                          &rawAngleData.m_frames[i * maxFrameLength], angleDataDTO);
        }
    }
}