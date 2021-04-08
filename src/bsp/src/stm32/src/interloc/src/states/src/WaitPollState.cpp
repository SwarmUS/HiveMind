#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/WaitPollState.h>

void WaitPollState::process(InterlocStateHandler& context) {
    Decawave& deca = InterlocBSPContainer::getDecawave(InterlocBSPContainer::DecawavePort::A);

    deca.receive(m_rxFrame, 0);

    if (m_rxFrame.m_status == UWBRxStatus::FINISHED &&
        reinterpret_cast<UWBMessages::DWFrame*>(m_rxFrame.m_rxBuffer)->m_functionCode ==
            UWBMessages::FunctionCode::TWR_POLL) {

        context.getTWR().m_pollRxTs = m_rxFrame.m_rxTimestamp;
        context.setState(InterlocStateContainer::getSendResponseState());
        return;
    }

    context.setState(InterlocStateContainer::getWaitPollState());
}
