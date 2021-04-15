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

    context.setState(InterlocStates::WAIT_RESPONSE, InterlocEvent::NO_EVENT);
}
