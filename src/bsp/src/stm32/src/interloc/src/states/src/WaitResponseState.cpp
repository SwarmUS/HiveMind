#include "Task.h"
#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/WaitResponseState.h>

WaitResponseState::WaitResponseState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void WaitResponseState::process(InterlocStateHandler& context) {
    //    uint64_t receiveStartTs =
    //        context.getTWR().m_pollTxTs + (POLL_TX_TO_RESP_RX_DLY_UUS * UUS_TO_DWT_TIME);

    //    m_decawaves[DecawavePort::A].receiveDelayed(m_rxFrame, 0,
    //                                                receiveStartTs); // TODO adjuste delays

    //    volatile uint64_t currentTime = m_decawaves[DecawavePort::A].getSysTime();
    //    volatile uint32_t timeUntilSend = (context.getTWR().m_pollTxTs - currentTime) /
    //    UUS_TO_DWT_TIME;

    m_decawaves[DecawavePort::A].receive(m_rxFrame, context.getTimeManager().getResponseTimeout());

    //    context.getTWR().m_finalTxTs =
    //        currentTime +
    //        (timeUntilSend + context.getTimeManager().getResponseTimeout()) * UUS_TO_DWT_TIME;

    // TODO add multiple receive logic
    if (m_rxFrame.m_status == UWBRxStatus::FINISHED &&
        reinterpret_cast<UWBMessages::DWFrame*>(m_rxFrame.m_rxBuffer.data())->m_functionCode ==
            UWBMessages::FunctionCode::TWR_RESPONSE) {

        m_decawaves[DecawavePort::A].getTxTimestamp(&context.getTWR().m_pollTxTs);
        context.getTWR()
            .m_responseRxTs[reinterpret_cast<UWBMessages::TWRResponse*>(m_rxFrame.m_rxBuffer.data())
                                ->m_subFrameId -
                            1] = m_rxFrame.m_rxTimestamp;

        context.setState(InterlocStates::SEND_FINAL, InterlocEvent::RX_LAST_RESP);
    } else if (m_rxFrame.m_status == UWBRxStatus::TIMEOUT) {
        // volatile uint64_t currentTime2 = m_decawaves[DecawavePort::A].getSysTime();
        context.setState(InterlocStates::SEND_FINAL, InterlocEvent::TIMEOUT);
    } else {
        // Wait a little and send next poll
        Task::delay(100);
        context.setState(InterlocStates::SEND_POLL, InterlocEvent::RX_ERROR);
    }
}
