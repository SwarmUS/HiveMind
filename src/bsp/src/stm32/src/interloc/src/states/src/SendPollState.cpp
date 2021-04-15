#include "states/SendPollState.h"
#include "interloc/InterlocBSPContainer.h"
#include "interloc/UWBMessages.h"
#include "states/InterlocStateContainer.h"
#include <Task.h>

SendPollState::SendPollState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void SendPollState::process(InterlocStateHandler& context) {
    Task::delay(10);
    context.constructUWBHeader(UWB_BROADCAST_ADDRESS, UWBMessages::BEACON, UWBMessages::TWR_POLL,
                               (uint8_t*)&m_pollMsg, sizeof(m_pollMsg));

    uint64_t txTime = m_decawaves[DecawavePort::A].getSysTime()+3000*UUS_TO_DWT_TIME;
    context.getTWR().m_pollTxTs = m_decawaves[DecawavePort::A].getTxTimestampFromDelayedTime(txTime);
    m_decawaves[DecawavePort::A].transmitDelayed((uint8_t*)&m_pollMsg, sizeof(UWBMessages::TWRPoll),txTime);

    context.setState(InterlocStates::WAIT_RESPONSE, InterlocEvent::NO_EVENT);
}
