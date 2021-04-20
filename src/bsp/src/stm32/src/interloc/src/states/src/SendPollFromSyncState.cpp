#include "states/SendPollFromSyncState.h"
#include <interloc/InterlocStateHandler.h>

SendPollFromSyncState::SendPollFromSyncState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void SendPollFromSyncState::process(InterlocStateHandler& context) {
    context.constructUWBHeader(UWB_BROADCAST_ADDRESS, UWBMessages::BEACON, UWBMessages::TWR_POLL,
                               (uint8_t*)&m_pollMsg, sizeof(m_pollMsg));
    m_pollMsg.m_currentFrameId = context.getSlotId();
    m_pollMsg.m_superFrameInitiator = context.getSlotId();

    m_decawaves[DecawavePort::A].transmit((uint8_t*)&m_pollMsg, sizeof(UWBMessages::TWRPoll));

    m_decawaves[DecawavePort::A].getTxTimestamp(&context.getTWR().m_pollTxTs);
    context.setLastFrameStartTs(context.getTWR().m_pollTxTs);

    context.setState(InterlocStates::WAIT_RESPONSE, InterlocEvent::NO_EVENT);
}
