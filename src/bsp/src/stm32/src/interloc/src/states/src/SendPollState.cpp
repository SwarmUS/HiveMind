#include "states/SendPollState.h"
#include "interloc/InterlocBSPContainer.h"

void SendPollState::process(InterlocStateHandler& context) {
    //    constructUWBHeader(destinationId, UWBMessages::BEACON, UWBMessages::TWR_POLL,
    //                       (uint8_t*)&pollMsg, sizeof(pollMsg));

    InterlocBSPContainer::getDecawave(InterlocBSPContainer::DecawavePort::A)
        .transmitAndReceiveDelayed((uint8_t*)(&m_pollMsg), sizeof(UWBMessages::TWRPoll),
                                   POLL_TX_TO_RESP_RX_DLY_UUS, m_responseMsg, RESP_RX_TIMEOUT_UUS);
}
