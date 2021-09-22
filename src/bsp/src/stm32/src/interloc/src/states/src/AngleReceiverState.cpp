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

    std::array<UWBRxFrame, numDecas> rxFrames;

    // TODO: Add some kind of timeout
    for (uint32_t i = 0; i < context.getAngleNumberOfFrames(); i++) {
        readAngleFrame(rxFrames);
        saveAngleData(context.getRawAngleData(), rxFrames, i);
    }
    context.getRawAngleData().m_framesLength = context.getAngleNumberOfFrames();

    if (InterlocBSPContainer::getInterlocManager().getState() ==
        InterlocStateDTO::ANGLE_CALIB_RECEIVER) {
        InterlocBSPContainer::getInterlocManager().sendRawAngleData(context.getRawAngleData());
    }

    context.setState(InterlocStates::IDLE, InterlocEvent::ANGLE_RECEIVED);
}

void AngleReceiverState::readAngleFrame(std::array<UWBRxFrame, numDecas>& rxFrames) {
    bool allDataReceived = true;
    do {
        // TODO: Map dynamically
        m_decawaves.getLeftAntenna()->get().receiveAsync(rxFrames[0], 0);
        m_decawaves.getRightAntenna()->get().receiveAsync(rxFrames[2], 0);
        m_decawaves.getMasterAntenna()->get().receive(rxFrames[1], 0);

        allDataReceived = true;
        for (auto& frame : rxFrames) {
            if (frame.m_status != UWBRxStatus::FINISHED) {
                allDataReceived = false;
            }

            auto* x = reinterpret_cast<UWBMessages::AngleMsg*>(frame.m_rxBuffer.data());
            x->m_headerFrame.m_functionCode = UWBMessages::ANGLE;
        }

    } while (!allDataReceived);
}

void AngleReceiverState::saveAngleData(BspInterlocRawAngleData& data,
                                       std::array<UWBRxFrame, numDecas>& rxFrames,
                                       uint32_t frameIndex) {
    data.m_frames[frameIndex].m_frameInfosLength = rxFrames.size();

    uint8_t j = 0;
    for (auto& frame : rxFrames) {
        data.m_frames[frameIndex].m_frameInfos[j].m_beeboardPort = j;
        data.m_frames[frameIndex].m_frameInfos[j].m_rxTimestamp = frame.m_rxTimestamp;
        data.m_frames[frameIndex].m_frameInfos[j].m_sfdAngle = frame.getSFDAngle();
        data.m_frames[frameIndex].m_frameInfos[j].m_accumulatorAngle = frame.getAccumulatorAngle();
        j++;
    }
}
