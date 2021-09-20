#include <interloc/InterlocBSPContainer.h>
#include <interloc/InterlocStateHandler.h>
#include <states/AngleReceiverState.h>

AngleReceiverState::AngleReceiverState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void AngleReceiverState::process(InterlocStateHandler& context) {
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
    // TODO: reactivate before using with BeeBoards
    (void)rxFrames;

    //    bool allDataReceived = true;
    //    do {
    //        // TODO: Map dynamically
    //        m_decawaves[DecawavePort::B].receiveAsync(rxFrames[1], 0);
    //        m_decawaves[DecawavePort::A].receive(rxFrames[0], 0);
    //
    //        allDataReceived = true;
    //        for (auto& frame : rxFrames) {
    //            if (frame.m_status != UWBRxStatus::FINISHED) {
    //                allDataReceived = false;
    //            }
    //        }
    //
    //    } while (!allDataReceived);
}

void AngleReceiverState::saveAngleData(BspInterlocRawAngleData& data,
                                       std::array<UWBRxFrame, numDecas>& rxFrames,
                                       uint32_t frameIndex) {
    data.m_frames[frameIndex].m_frameInfosLength = rxFrames.size();
    // TODO: reactivate before using with BeeBoards

    //    uint8_t j = 0;
    //    for (auto& frame : rxFrames) {
    //        data.m_frames[frameIndex].m_frameInfos[j].m_beeboardPort = j;
    //        data.m_frames[frameIndex].m_frameInfos[j].m_rxTimestamp = frame.m_rxTimestamp;
    //        data.m_frames[frameIndex].m_frameInfos[j].m_sfdAngle = frame.getSFDAngle();
    //        data.m_frames[frameIndex].m_frameInfos[j].m_accumulatorAngle =
    //        frame.getAccumulatorAngle(); j++;
    //    }

    uint8_t j = 0;
    for (auto& frame : rxFrames) {
        data.m_frames[frameIndex].m_frameInfos[j].m_beeboardPort = j;
        data.m_frames[frameIndex].m_frameInfos[j].m_rxTimestamp = frameIndex + j;
        data.m_frames[frameIndex].m_frameInfos[j].m_sfdAngle = frameIndex + 1 + j;
        data.m_frames[frameIndex].m_frameInfos[j].m_accumulatorAngle = frameIndex + 2 + j;
        j++;
    }
}
