#include "states/SendPollState.h"
#include "interloc/InterlocBSPContainer.h"
#include "interloc/UWBMessages.h"
#include "states/InterlocStateContainer.h"
#include <Task.h>

void SendPollState::process(InterlocStateHandler& context) {
    InterlocBSPContainer::getInterlocManager().constructUWBHeader(
        UWB_BROADCAST_ADDRESS, UWBMessages::BEACON, UWBMessages::TWR_POLL, (uint8_t*)&m_pollMsg,
        sizeof(m_pollMsg));

    Decawave& deca = InterlocBSPContainer::getDecawave(InterlocBSPContainer::DecawavePort::A);

    deca.transmitAndReceiveDelayed((uint8_t*)&m_pollMsg, sizeof(UWBMessages::TWRPoll),
                                   POLL_TX_TO_RESP_RX_DLY_UUS, m_responseFrame,
                                   RESP_RX_TIMEOUT_UUS);

    if (m_responseFrame.m_status == UWBRxStatus::FINISHED &&
        reinterpret_cast<UWBMessages::DWFrame*>(m_responseFrame.m_rxBuffer)->m_functionCode ==
            UWBMessages::FunctionCode::TWR_RESPONSE) {

        deca.getTxTimestamp(&context.getTWR().m_pollTxTs);
        deca.getRxTimestamp(&context.getTWR().m_responseRxTs);

        context.setState(InterlocStateContainer::getSendFinalState());
    } else {
        // Wait a little and send next poll
        Task::delay(100);
        context.setState(InterlocStateContainer::getSendPollState());
    }
}
