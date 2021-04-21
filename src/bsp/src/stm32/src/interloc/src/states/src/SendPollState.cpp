#include "states/SendPollState.h"
#include "interloc/InterlocBSPContainer.h"
#include "interloc/UWBMessages.h"
#include "states/InterlocStateContainer.h"

SendPollState::SendPollState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void SendPollState::process(InterlocStateHandler& context) {
    context.constructUWBHeader(UWB_BROADCAST_ADDRESS, UWBMessages::BEACON, UWBMessages::TWR_POLL,
                               (uint8_t*)&m_pollMsg, sizeof(m_pollMsg));
    m_pollMsg.m_currentFrameId = context.getSlotId();
    m_pollMsg.m_superFrameInitiator = context.getSuperFrameInitiator();

    uint64_t txTime = context.getTimeManager().getPollTxTs(context.getPreviousFrameStartTs());

    context.getTWR().m_pollTxTs =
        m_decawaves[DecawavePort::A].getTxTimestampFromDelayedTime(txTime);
    context.setPreviousFrameStartTs(context.getTWR().m_pollTxTs);

    m_decawaves[DecawavePort::A].transmitDelayed((uint8_t*)&m_pollMsg, sizeof(UWBMessages::TWRPoll),
                                                 txTime);

    context.setState(InterlocStates::WAIT_RESPONSE, InterlocEvent::NO_EVENT);
}
