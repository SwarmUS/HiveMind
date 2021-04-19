#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/WaitPollState.h>

WaitPollState::WaitPollState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void WaitPollState::process(InterlocStateHandler& context) {
    m_decawaves[DecawavePort::A].receive(m_rxFrame, context.getTimeManager().getPollTimeout());
    //    m_decawaves[DecawavePort::A].receive(m_rxFrame, 0);

    if (m_rxFrame.m_status == UWBRxStatus::FINISHED &&
        reinterpret_cast<UWBMessages::DWFrame*>(m_rxFrame.m_rxBuffer.data())->m_functionCode ==
            UWBMessages::FunctionCode::TWR_POLL) {

        context.getTWR().m_pollRxTs = m_rxFrame.m_rxTimestamp;
        context.setState(InterlocStates::SEND_RESPONSE, InterlocEvent::POLL_RECVD);
        return;
    }

    context.setState(InterlocStates::SEND_POLL, InterlocEvent::TIMEOUT);
}
