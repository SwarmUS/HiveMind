#include <interloc/InterlocBSPContainer.h>
#include <interloc/InterlocStateHandler.h>
#include <states/AngleReceiverState.h>

AngleReceiverState::AngleReceiverState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void AngleReceiverState::process(InterlocStateHandler& context) {
    if (!m_decawaves.canCalculateAngles()) {
        context.setState(InterlocStates::IDLE, InterlocEvent::DECA_INIT_ERROR);
        return;
    }
    InterlocStateDTO managerState = InterlocBSPContainer::getInterlocManager().getState();

    uint32_t receivedFrames = 0;
    uint32_t iterations = 0;
    uint32_t maxIterations =
        managerState == InterlocStateDTO::ANGLE_CALIB_RECEIVER ? 500 : NUM_ANGLE_MSG / 2;
    uint16_t timeout =
        InterlocTimeManager::getTimeoutUs(context.getTimeManager().m_angleAirTimeWithPreambleUs);

    while (iterations < maxIterations && receivedFrames < context.getAngleNumberOfFrames()) {
        bool allDataReceived = false;

        if (managerState == InterlocStateDTO::ANGLE_CALIB_RECEIVER) {
            allDataReceived = readAngleFrameContinuousMode();
        } else {
            allDataReceived = readAngleFrameNormalMode(context, iterations, timeout);
        }

        if (allDataReceived) {
            saveAngleData(context.getRawAngleData(), receivedFrames);
            receivedFrames++;
        }
        iterations++;
    }

    context.getRawAngleData().m_framesLength = context.getAngleNumberOfFrames();

    if (InterlocBSPContainer::getInterlocManager().getState() ==
        InterlocStateDTO::ANGLE_CALIB_RECEIVER) {
        InterlocBSPContainer::getInterlocManager().sendRawAngleData(context.getRawAngleData());
        context.setState(InterlocStates::IDLE, InterlocEvent::ANGLE_RECEIVED);
    }

    context.setState(InterlocStates::CALCULATE_INTERLOC, InterlocEvent::ANGLE_RECEIVED);
}

bool AngleReceiverState::readAngleFrameContinuousMode() {
    for (auto deca : m_decawaves.getAngleAntennaArray()) {
        deca->get().receiveAsync(0);
    }

    if (waitReceptionOrTimeout()) {
        return verifyDataValid();
    }

    return false;
}

bool AngleReceiverState::readAngleFrameNormalMode(const InterlocStateHandler& context,
                                                  uint32_t angleId,
                                                  uint16_t timeoutUs) {
    uint64_t rxStartTime =
        context.getTimeManager().getAngleRxStartTs(context.getPreviousFrameStartTs(), angleId);
    volatile uint64_t currentTs = m_decawaves.getMasterAntenna()->get().getSysTime();

    for (auto deca : m_decawaves.getAngleAntennaArray()) {
        deca->get().receiveAsyncDelayed(timeoutUs, rxStartTime);
    }

    if (waitReceptionOrTimeout()) {
        return verifyDataValid();
    }

    return false;
}

bool AngleReceiverState::verifyDataValid() {
    for (uint8_t i = 0; i < m_decawaves.getAngleAntennaArray().size(); i++) {
        m_decawaves.getAngleAntennaArray()[i]->get().retrieveRxFrame(m_rxFrames[i]);
        if (reinterpret_cast<UWBMessages::AngleMsg*>(m_rxFrames[i].m_rxBuffer.data())
                ->m_headerFrame.m_functionCode != UWBMessages::ANGLE) {
            return false;
        }
    }

    return true;
}

bool AngleReceiverState::waitReceptionOrTimeout() {
    bool allDataReceived;
    m_decawaves.getMasterAntenna()->get().awaitRx();
    allDataReceived = true;

    // Verify all decawaves received a message
    for (auto deca : m_decawaves.getAngleAntennaArray()) {
        UWBRxStatus status = deca->get().getRxStatus();

        if (status == UWBRxStatus::ONGOING) {
            status = deca->get().awaitRx();
        }

        if (status != UWBRxStatus::FINISHED) {
            allDataReceived = false;
        }
    }

    return allDataReceived;
}

void AngleReceiverState::saveAngleData(BspInterlocRawAngleData& data, uint32_t frameIndex) {
    data.m_frames[frameIndex].m_frameInfosLength = m_rxFrames.size();

    for (uint8_t i = 0; i < m_rxFrames.size(); i++) {
        data.m_frames[frameIndex].m_frameInfos[i].m_beeboardPort = i;
        data.m_frames[frameIndex].m_frameInfos[i].m_rxTimestamp = m_rxFrames[i].m_rxTimestamp;
        data.m_frames[frameIndex].m_frameInfos[i].m_sfdAngle = m_rxFrames[i].getSFDAngle();
        data.m_frames[frameIndex].m_frameInfos[i].m_accumulatorAngle =
            m_rxFrames[i].getAccumulatorAngle();
        data.m_frames[frameIndex].m_frameInfos[i].m_messageId =
            reinterpret_cast<UWBMessages::AngleMsg*>(m_rxFrames[i].m_rxBuffer.data())->m_messageId;
    }
}
