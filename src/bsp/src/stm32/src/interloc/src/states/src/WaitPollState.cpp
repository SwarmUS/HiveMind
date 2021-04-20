#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/WaitPollState.h>

WaitPollState::WaitPollState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void WaitPollState::process(InterlocStateHandler& context) {
    uint64_t rxStartTs = context.getTimeManager().getPollRxStartTs(context.getLastFrameStartTs());
    m_decawaves[DecawavePort::A].receiveDelayed(
        m_rxFrame, context.getTimeManager().getPollTimeout(), rxStartTs);
    // m_decawaves[DecawavePort::A].receive(m_rxFrame,
    // context.getTimeManager().getPollTimeout());

    if (m_rxFrame.m_status == UWBRxStatus::FINISHED && DecawaveUtils::isFramePoll(m_rxFrame)) {

        context.getTWR().m_pollRxTs = m_rxFrame.m_rxTimestamp;
        context.setLastFrameStartTs(m_rxFrame.m_rxTimestamp);

        context.setState(InterlocStates::SEND_RESPONSE, InterlocEvent::POLL_RECVD);
        return;
    }

    context.setState(InterlocStates::IDLE, InterlocEvent::TIMEOUT);
}
