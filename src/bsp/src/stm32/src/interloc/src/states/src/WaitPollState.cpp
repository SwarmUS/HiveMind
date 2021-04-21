#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/WaitPollState.h>

WaitPollState::WaitPollState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void WaitPollState::process(InterlocStateHandler& context) {
    uint64_t rxStartTs =
        context.getTimeManager().getPollRxStartTs(context.getPreviousFrameStartTs());
    m_decawaves[DecawavePort::A].receiveDelayed(
        m_rxFrame, context.getTimeManager().getPollTimeout(), rxStartTs);

    if (m_rxFrame.m_status == UWBRxStatus::FINISHED && DecawaveUtils::isFramePoll(m_rxFrame)) {

        context.getTWR().m_pollRxTs = m_rxFrame.m_rxTimestamp;
        context.setPreviousFrameStartTs(m_rxFrame.m_rxTimestamp);

        context.setState(InterlocStates::SEND_RESPONSE, InterlocEvent::POLL_RECVD);
        return;
    }

    context.setPreviousFrameStartTs(rxStartTs + RX_BEFORE_TX_GUARD_US * UUS_TO_DWT_TIME);
    context.setState(InterlocStates::IDLE, InterlocEvent::TIMEOUT);
}
