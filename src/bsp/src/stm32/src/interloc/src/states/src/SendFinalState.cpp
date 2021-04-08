#include "states/SendFinalState.h"
#include "interloc/InterlocStateHandler.h"
#include <Task.h>
#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>

void SendFinalState::process(InterlocStateHandler& context) {
    Decawave& deca = InterlocBSPContainer::getDecawave(InterlocBSPContainer::DecawavePort::A);

    uint64_t finalTxTime =
        context.getTWR().m_responseRxTs + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME);

    // Get the timestamp at which it will really be sent
    uint64_t finalTxTs = deca.getTxTimestampFromDelayedTime(finalTxTime);

    // TODO: ADD DESTINATION ID
    InterlocBSPContainer::getInterlocManager().constructUWBHeader(
        0x01, UWBMessages::DATA, UWBMessages::TWR_FINAL, (uint8_t*)(&m_finalMsg),
        sizeof(m_finalMsg));

    context.getTWR().constructFinal(&m_finalMsg, finalTxTs);
    deca.transmitDelayed((uint8_t*)(&m_finalMsg), sizeof(UWBMessages::TWRFinal), finalTxTime);

    Task::delay(100);
    context.setState(InterlocStateContainer::getSendPollState());
}
