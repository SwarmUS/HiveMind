#include <hal/hal_timer.h>
#include <interloc/InterlocBSPContainer.h>
#include <interloc/InterlocStateHandler.h>
#include <states/AngleReceiverState.h>

void AngleReceiverState::staticTimerCallback(void* context) {
    static_cast<AngleReceiverState*>(context)->timerCallback();
}

void AngleReceiverState::timerCallback() {
    if (m_timeoutHundredMicros > 0) {
        m_timeoutHundredMicros--;
    } else {
        Timer_setHundredMicrosCallback(nullptr, nullptr);
        abortRx();
    }
}

AngleReceiverState::AngleReceiverState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void AngleReceiverState::process(InterlocStateHandler& context) {
    if (!m_decawaves.canCalculateAngles()) {
        context.setState(InterlocStates::CALCULATE_INTERLOC, InterlocEvent::DECA_INIT_ERROR);
        return;
    }
    InterlocStateDTO managerState = InterlocBSPContainer::getInterlocManager().getState();

    uint32_t receivedFrames = 0;
    // Here for characterization and tuning
    volatile uint32_t lostFrames = 0;
    uint32_t maxIterations = managerState == InterlocStateDTO::ANGLE_CALIB_RECEIVER
                                 ? context.getAngleCalibNumberOfFrames()
                                 : NUM_ANGLE_MSG_RECEIVER;

    if (managerState != InterlocStateDTO::ANGLE_CALIB_RECEIVER) {
        uint64_t rxStopTime =
            context.getTimeManager().getAngleRxStopTs(context.getPreviousFrameStartTs());
        uint64_t currentTs = m_decawaves.getMasterAntenna()->get().getSysTime();
        m_timeoutHundredMicros = ((rxStopTime - currentTs) % UINT40_MAX) / UUS_TO_DWT_TIME / 100;
        m_aborted = false;
        Timer_setHundredMicrosCallback(staticTimerCallback, this);
    } else {
        m_timeoutHundredMicros =
            context.getTimeManager().getAngleToAngleOffsetUs() * maxIterations * 2;
        m_aborted = false;
        Timer_setHundredMicrosCallback(staticTimerCallback, this);
    }

    while (receivedFrames < maxIterations && !m_aborted) {
        bool allDataReceived = false;

        allDataReceived = readAngleFrame();

        if (allDataReceived) {
            saveAngleData(context.getRawAngleData(), receivedFrames);
            receivedFrames++;
        } else {
            lostFrames++;
        }
    }

    context.getRawAngleData().m_framesLength = receivedFrames;

    if (InterlocBSPContainer::getInterlocManager().getState() ==
        InterlocStateDTO::ANGLE_CALIB_RECEIVER) {
        InterlocBSPContainer::getInterlocManager().sendRawAngleData(context.getRawAngleData());
        context.setState(InterlocStates::IDLE, InterlocEvent::ANGLE_RECEIVED);
    }

    context.setState(InterlocStates::CALCULATE_INTERLOC, InterlocEvent::ANGLE_RECEIVED);
}

bool AngleReceiverState::readAngleFrame() {
    for (auto deca : m_decawaves.getAngleAntennaArray()) {
        deca->get().receiveAsync(0);
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

void AngleReceiverState::abortRx() {
    m_aborted = true;
    for (auto deca : m_decawaves.getAngleAntennaArray()) {
        deca->get().abortTRXFromISR();
    }
}
