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

    uint64_t txTime = context.getTimeManager().getPollTxStartTs(
        context.getPreviousFrameStartTs() +
        (uint64_t)(UUS_TO_DWT_TIME * InterlocTimeManager::getPreambleAirTimeUs()));

    uint64_t pollTxTs = m_decawaves[DecawavePort::A].getTxTimestampFromDelayedTime(txTime);

    context.getTWR().m_pollTxTs = pollTxTs;

    context.setPreviousFrameStartTs(context.getTWR().m_pollTxTs);

    m_decawaves[DecawavePort::A].transmitDelayed((uint8_t*)&m_pollMsg, sizeof(UWBMessages::TWRPoll),
                                                 pollTxTs);
    // set the RxTs values for the wait_response state
    context.getTimeManager().computeResponseRxTs(context.getPreviousFrameStartTs());
    context.setState(InterlocStates::WAIT_RESPONSE, InterlocEvent::NO_EVENT);
}
