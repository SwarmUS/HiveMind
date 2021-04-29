#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/WaitPollState.h>

WaitPollState::WaitPollState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void WaitPollState::process(InterlocStateHandler& context) {
    //    uint64_t rxStartTs =
    //        context.getTimeManager().getPollRxStartTs(context.getPreviousFrameStartTs());
    uint64_t rxStartTs =
        context.getTimeManager().getPollRxStartTs_new(context.getPreviousFrameStartTs());

    m_decawaves[DecawavePort::A].receiveDelayed(
        m_rxFrame,
        InterlocTimeManager::getTimeoutUs_new(context.getTimeManager().m_pollAirTimeWithPreamble),
        rxStartTs);

    if (m_rxFrame.m_status == UWBRxStatus::FINISHED && DecawaveUtils::isFramePoll(m_rxFrame)) {

        context.getTWR().m_pollRxTs = m_rxFrame.m_rxTimestamp;

        // sychronisation parameters
        context.setPreviousFrameStartTs(m_rxFrame.m_rxTimestamp);
        context.setSuperFrameInitiator(
            reinterpret_cast<UWBMessages::TWRPoll*>(m_rxFrame.m_rxBuffer.data())
                ->m_superFrameInitiator);
        context.setCurrentFrameId(
            reinterpret_cast<UWBMessages::TWRPoll*>(m_rxFrame.m_rxBuffer.data())->m_currentFrameId);

        context.setState(InterlocStates::SEND_RESPONSE, InterlocEvent::POLL_RECVD);
        return;
    }

    context.setPreviousFrameStartTs(rxStartTs + RX_BEFORE_TX_GUARD_US * UUS_TO_DWT_TIME);
    context.setState(InterlocStates::IDLE, InterlocEvent::TIMEOUT);
}
