#include "states/SendPollFromSyncState.h"
#include <interloc/InterlocStateHandler.h>

SendPollFromSyncState::SendPollFromSyncState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void SendPollFromSyncState::process(InterlocStateHandler& context) {
    if (auto deca = m_decawaves.getMasterAntenna()) {
        context.constructUWBHeader(UWB_BROADCAST_ADDRESS, UWBMessages::BEACON,
                                   UWBMessages::TWR_POLL, (uint8_t*)&m_pollMsg, sizeof(m_pollMsg));
        m_pollMsg.m_currentFrameId = context.getSlotId();
        m_pollMsg.m_superFrameInitiator = context.getSlotId();

        context.setSuperFrameInitiator(context.getSlotId());
        context.setCurrentFrameId(context.getSlotId());

        deca->get().transmit((uint8_t*)&m_pollMsg, sizeof(UWBMessages::TWRPoll));

        deca->get().getTxTimestamp(&context.getTWR().m_pollTxTs);
        context.getTWR().m_pollTxTs = context.getTWR().m_pollTxTs + deca->get().getTxAntennaDLY();
        context.setPreviousFrameStartTs(context.getTWR().m_pollTxTs);

        // set the RxTs values for the wait_response state
        context.getTimeManager().computeResponseRxTs(context.getPreviousFrameStartTs());
        context.setState(InterlocStates::WAIT_RESPONSE, InterlocEvent::NO_EVENT);
    } else {
        context.setState(InterlocStates::IDLE, InterlocEvent::DECA_INIT_ERROR);
    }
}
