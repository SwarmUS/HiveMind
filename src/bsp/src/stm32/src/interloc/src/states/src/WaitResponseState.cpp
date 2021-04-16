#include "Task.h"
#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/WaitResponseState.h>

WaitResponseState::WaitResponseState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void WaitResponseState::process(InterlocStateHandler& context) {
    m_decawaves[DecawavePort::A].receive(m_rxFrame, context.getTimeManager().getResponseTimeout());

    // TODO add multiple receive logic
    if (m_rxFrame.m_status == UWBRxStatus::FINISHED &&
        reinterpret_cast<UWBMessages::DWFrame*>(m_rxFrame.m_rxBuffer.data())->m_functionCode ==
            UWBMessages::FunctionCode::TWR_RESPONSE) {

        m_decawaves[DecawavePort::A].getTxTimestamp(&context.getTWR().m_pollTxTs);
        context.getTWR()
            .m_responseRxTs[reinterpret_cast<UWBMessages::TWRResponse*>(m_rxFrame.m_rxBuffer.data())
                                ->m_subFrameId -
                            1] = m_rxFrame.m_rxTimestamp;

        context.setState(InterlocStates::SEND_FINAL, InterlocEvent::RX_LAST_RESP);
    } else if (m_rxFrame.m_status == UWBRxStatus::TIMEOUT) {
        context.setState(InterlocStates::SEND_FINAL, InterlocEvent::TIMEOUT);
    } else {
        // Wait a little and send next poll
        Task::delay(100);
        context.setState(InterlocStates::SEND_POLL, InterlocEvent::RX_ERROR);
    }
}
