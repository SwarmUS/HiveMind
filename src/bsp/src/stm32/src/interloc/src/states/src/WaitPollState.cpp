#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/WaitPollState.h>

WaitPollState::WaitPollState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void WaitPollState::process(InterlocStateHandler& context) {
    if (auto deca = m_decawaves.getMasterAntenna()) {
        uint64_t rxStartTs =
            context.getTimeManager().getPollRxStartTs(context.getPreviousFrameStartTs());

        deca->get().receiveDelayed(
            m_rxFrame,
            InterlocTimeManager::getTimeoutUs(context.getTimeManager().m_pollAirTimeWithPreambleUs),
            rxStartTs);

        if (m_rxFrame.m_status == UWBRxStatus::FINISHED && DecawaveUtils::isFramePoll(m_rxFrame)) {

            UWBMessages::TWRPoll* msg =
                reinterpret_cast<UWBMessages::TWRPoll*>(m_rxFrame.m_rxBuffer.data());

            context.getTWR().m_pollRxTs = m_rxFrame.m_rxTimestamp;
            context.setPreviousFrameStartTs(m_rxFrame.m_rxTimestamp);
            context.setSuperFrameInitiator(msg->m_superFrameInitiator);
            context.setCurrentFrameId(msg->m_currentFrameId);

            context.setState(InterlocStates::SEND_RESPONSE, InterlocEvent::POLL_RECVD);
            return;
        }
        context.setPreviousFrameStartTs(rxStartTs +
                                        (RX_BEFORE_TX_GUARD_US +
                                         (uint64_t)context.getTimeManager().getPreambleAirTimeUs() +
                                         POLL_PROCESSING_GUARD) *
                                            UUS_TO_DWT_TIME);
        context.setState(InterlocStates::IDLE, InterlocEvent::TIMEOUT);
    } else {
        context.setState(InterlocStates::IDLE, InterlocEvent::DECA_INIT_ERROR);
    }
}
