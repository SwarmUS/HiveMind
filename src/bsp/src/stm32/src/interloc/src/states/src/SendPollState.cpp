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
    // moment to start the transmit
    volatile uint64_t txTime = context.getTimeManager().getPollTxStartTs(
        context.getPreviousFrameStartTs() + 70 * UUS_TO_DWT_TIME);
    // moment the actual message is transmit
    volatile uint64_t pollTxTs = m_decawaves[DecawavePort::A].getTxTimestampFromDelayedTime(txTime);

    context.getTWR().m_pollTxTs = pollTxTs;

    m_decawaves[DecawavePort::A].transmitDelayed((uint8_t*)&m_pollMsg, sizeof(UWBMessages::TWRPoll),
                                                 txTime);

    m_decawaves[DecawavePort::A].getTxTimestamp(&context.getTWR().m_pollTxTs);
    context.setPreviousFrameStartTs(context.getTWR().m_pollTxTs);

    // set the RxTs values for the wait_response state
    context.getTimeManager().computeResponseRxTs(context.getPreviousFrameStartTs());
    context.setState(InterlocStates::WAIT_RESPONSE, InterlocEvent::NO_EVENT);
}
