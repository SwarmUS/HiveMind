#include "states/SendPollState.h"
#include "interloc/InterlocBSPContainer.h"
#include "interloc/UWBMessages.h"
#include "states/InterlocStateContainer.h"
#include <Task.h>

SendPollState::SendPollState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void SendPollState::process(InterlocStateHandler& context) {
    context.constructUWBHeader(UWB_BROADCAST_ADDRESS, UWBMessages::BEACON, UWBMessages::TWR_POLL,
                               (uint8_t*)&m_pollMsg, sizeof(m_pollMsg));

    m_decawaves[DecawavePort::A].transmitAndReceiveDelayed(
        (uint8_t*)&m_pollMsg, sizeof(UWBMessages::TWRPoll), POLL_TX_TO_RESP_RX_DLY_UUS,
        m_responseFrame, RESP_RX_TIMEOUT_UUS);

    if (m_responseFrame.m_status == UWBRxStatus::FINISHED &&
        reinterpret_cast<UWBMessages::DWFrame*>(m_responseFrame.m_rxBuffer.data())
                ->m_functionCode == UWBMessages::FunctionCode::TWR_RESPONSE) {

        m_decawaves[DecawavePort::A].getTxTimestamp(&context.getTWR().m_pollTxTs);
        context.getTWR().m_responseRxTs[reinterpret_cast<UWBMessages::TWRResponse*>(
                                            m_responseFrame.m_rxBuffer.data())
                                            ->m_subFrameId] = m_responseFrame.m_rxTimestamp;

        context.setState(InterlocStates::SEND_FINAL);
    } else {
        // Wait a little and send next poll
        Task::delay(100);
        context.setState(InterlocStates::SEND_POLL);
    }
}
