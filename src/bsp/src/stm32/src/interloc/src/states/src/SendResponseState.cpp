#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/SendResponseState.h>

void SendResponseState::process(InterlocStateHandler& context) {
    Decawave& deca = InterlocBSPContainer::getDecawave(InterlocBSPContainer::DecawavePort::A);

    // TODO: Set destination ID
    InterlocBSPContainer::getInterlocManager().constructUWBHeader(
        0x01, UWBMessages::DATA, UWBMessages::TWR_RESPONSE, (uint8_t*)&m_respMsg,
        sizeof(m_respMsg));

    uint64_t respTxTime =
        context.getTWR().m_pollRxTs + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME);
    // deca.transmitAndReceive((uint8_t*)&m_respMsg, sizeof(m_respMsg), 0, m_rxFrame, 0);
    deca.transmitDelayedAndReceive((uint8_t*)&m_respMsg, sizeof(m_respMsg), respTxTime, 0,
                                   m_rxFrame, FINAL_RX_TIMEOUT_UUS);

    // Get the real time at which the response was be sent
    context.getTWR().m_responseTxTs = deca.getTxTimestampFromDelayedTime(respTxTime);

    if (m_rxFrame.m_status == UWBRxStatus::FINISHED &&
        reinterpret_cast<UWBMessages::DWFrame*>(m_rxFrame.m_rxBuffer)->m_functionCode ==
            UWBMessages::FunctionCode::TWR_FINAL) {

        context.getTWR().m_finalRxTs = m_rxFrame.m_rxTimestamp;
        context.getTWR().deserializeFinal(
            reinterpret_cast<UWBMessages::TWRFinal*>(m_rxFrame.m_rxBuffer));
        double distance = context.getTWR().calculateDistance();
        printf("%f\n", distance);
    }

    context.setState(InterlocStateContainer::getWaitPollState());
}
