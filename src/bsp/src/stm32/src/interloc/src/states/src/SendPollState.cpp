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

    m_decawaves[DecawavePort::A].transmit( (uint8_t*)&m_pollMsg, sizeof(UWBMessages::TWRPoll));

    if (m_responseFrame.m_status == UWBRxStatus::FINISHED &&
        reinterpret_cast<UWBMessages::DWFrame*>(m_responseFrame.m_rxBuffer.data())
                ->m_functionCode == UWBMessages::FunctionCode::TWR_RESPONSE) {

        m_decawaves[DecawavePort::A].getTxTimestamp(&context.getTWR().m_pollTxTs);
        context.getTWR().m_responseRxTs[reinterpret_cast<UWBMessages::TWRResponse*>(
                                            m_responseFrame.m_rxBuffer.data())
                                            ->m_subFrameId -
                                        1] = m_responseFrame.m_rxTimestamp;

        context.setState(InterlocStates::SEND_FINAL, InterlocEvent::RESPONSE_RECVD);
    } else {
        // Wait a little and send next poll
        Task::delay(100);
        context.setState(InterlocStates::SEND_POLL, InterlocEvent::TIMEOUT);
    }
    // go to wait response
}
