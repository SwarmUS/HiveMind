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

    // TODO: Add some kind of timeout
    for (uint32_t i = 0; i < context.getAngleNumberOfFrames(); i++) {
        readAngleFrame();
        saveAngleData(context.getRawAngleData(), i);
    }
    context.getRawAngleData().m_framesLength = context.getAngleNumberOfFrames();

    if (InterlocBSPContainer::getInterlocManager().getState() ==
        InterlocStateDTO::ANGLE_CALIB_RECEIVER) {
        InterlocBSPContainer::getInterlocManager().sendRawAngleData(context.getRawAngleData());
    }

    context.setState(InterlocStates::IDLE, InterlocEvent::ANGLE_RECEIVED);
}

void AngleReceiverState::readAngleFrame() {
    bool allDataReceived;
    do {
        for (auto deca : m_decawaves.getAngleAntennaArray()) {
            deca.get().receiveAsync(0);
        }

        m_decawaves.getMasterAntenna()->get().awaitRx();
        allDataReceived = true;

        // Verify all decawaves received a message
        for (auto deca : m_decawaves.getAngleAntennaArray()) {
            if (deca.get().getRxStatus() != UWBRxStatus::FINISHED) {
                allDataReceived = false;
            }
        }

        // If all decawaves received a frame, extract data from the internal registers (if one of
        // the received messages was not angle message, abort)
        if (allDataReceived) {
            for (uint8_t i = 0; i < m_decawaves.getAngleAntennaArray().size(); i++) {
                m_decawaves.getAngleAntennaArray()[i].get().retrieveRxFrame(m_rxFrames[i]);
                if (reinterpret_cast<UWBMessages::AngleMsg*>(m_rxFrames[i].m_rxBuffer.data())
                        ->m_headerFrame.m_functionCode != UWBMessages::ANGLE) {
                    allDataReceived = false;
                    break;
                }
            }
        }
    } while (!allDataReceived);
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
