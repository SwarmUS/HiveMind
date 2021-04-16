#include "Task.h"
#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/WaitResponseState.h>
#include <states/InterlocStateContainer.h>

WaitResponseState::WaitResponseState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void WaitResponseState::process(InterlocStateHandler& context) {
    m_decawaves[DecawavePort::A].receive(m_rxFrame, InterlocTimeManager::getResponseTimeout());

    // TODO add multiple receive logic
    if (m_rxFrame.m_status == UWBRxStatus::FINISHED &&
        reinterpret_cast<UWBMessages::DWFrame*>(m_rxFrame.m_rxBuffer.data())->m_functionCode ==
            UWBMessages::FunctionCode::TWR_RESPONSE) {

        m_decawaves[DecawavePort::A].getTxTimestamp(&context.getTWR().m_pollTxTs);
        context.getTWR()
            .m_responseRxTs[index] = m_rxFrame.m_rxTimestamp;

        nbTries ++;

        if (index == MAX_INTERLOC_SUBFRAMES || nbTries >= MAX_INTERLOC_SUBFRAMES) {
            nbTries ++;
            context.setState(InterlocStates::SEND_FINAL, InterlocEvent::RX_LAST_RESP);
        // this was not the last expected Rx, continue to receive
        }
        context.setState(InterlocStates::WAIT_RESPONSE, InterlocEvent::RX_RESP_GOOD);
    } else if (m_rxFrame.m_status == UWBRxStatus::TIMEOUT) {
        if(nbTries >= MAX_INTERLOC_SUBFRAMES){

        context.getTWR().m_responseRxTs[nbTries ++] = m_rxFrame.m_rxTimestamp;
            nbTries = 0;
        }
            context.setState(InterlocStates::SEND_FINAL, InterlocEvent::RX_LAST_TIMEOUT);
        context.setState(InterlocStates::WAIT_RESPONSE, InterlocEvent::TIMEOUT);
    } else {
        // Wait a little and send next poll
        Task::delay(100);
        context.setState(InterlocStates::SEND_POLL, InterlocEvent::RX_ERROR);
        while(1){

        }
    }
}