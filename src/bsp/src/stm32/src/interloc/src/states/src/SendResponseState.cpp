#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/SendResponseState.h>

SendResponseState::SendResponseState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void SendResponseState::process(InterlocStateHandler& context) {
    // TODO: Set destination ID
    context.constructUWBHeader(0x01, UWBMessages::DATA, UWBMessages::TWR_RESPONSE,
                               (uint8_t*)&m_respMsg, sizeof(m_respMsg));
    m_respMsg.m_subFrameId = context.getSlotId();

    uint64_t respTxTime = context.getTimeManager().getResponseTxTs(context.getTWR().m_pollRxTs);

    m_decawaves[DecawavePort::A].transmitDelayed((uint8_t*)&m_respMsg, sizeof(m_respMsg),
                                                 respTxTime);

    // Get the real time at which the response was sent
    context.getTWR().m_responseTxTs =
        m_decawaves[DecawavePort::A].getTxTimestampFromDelayedTime(respTxTime);

    context.setState(InterlocStates::WAIT_FINAL, InterlocEvent::NO_EVENT);
}
