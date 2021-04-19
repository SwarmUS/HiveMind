#include "Task.h"
#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/WaitResponseState.h>

WaitResponseState::WaitResponseState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void WaitResponseState::process(InterlocStateHandler& context) {
    uint64_t pollTxTs = 0;
    m_decawaves[DecawavePort::A].getTxTimestamp(&pollTxTs);
    //        volatile uint64_t startRxTime = InterlocTimeManager::getRespRxStartTime(pollTxTs,
    //        nbTries); m_decawaves[DecawavePort::A].receiveDelayed(
    //            m_rxFrame, InterlocTimeManager::getResponseTimeout(), startRxTime);

    //    volatile uint64_t var1 = m_decawaves[DecawavePort::A].getSysTime();
    //    var1++;
    //    volatile uint64_t var2 = InterlocTimeManager::getResponseTimeout(nbTries);
    //    var2++;

    m_decawaves[DecawavePort::A].receive(m_rxFrame, InterlocTimeManager::getResponseTimeout());

    // TODO add multiple receive logic
    if (m_rxFrame.m_status == UWBRxStatus::FINISHED &&
        reinterpret_cast<UWBMessages::DWFrame*>(m_rxFrame.m_rxBuffer.data())->m_functionCode ==
            UWBMessages::FunctionCode::TWR_RESPONSE) {

        m_decawaves[DecawavePort::A].getTxTimestamp(&context.getTWR().m_pollTxTs);
        uint8_t index =
            reinterpret_cast<UWBMessages::TWRResponse*>(m_rxFrame.m_rxBuffer.data())->m_subFrameId -
            1;
        context.getTWR().m_responseRxTs[index] = m_rxFrame.m_rxTimestamp;
        //        m_logger.log(LogLevel::Warn, "got response %d", index);
        if (index == MAX_INTERLOC_SUBFRAMES - 1) {
            // Last Response, sendFinal to all
            nbTries = 0;
            context.setState(InterlocStates::SEND_FINAL, InterlocEvent::RX_LAST_RESP);
            return;
        }
        // this was not the last expected Rx, continue to receive
        context.setState(InterlocStates::WAIT_RESPONSE, InterlocEvent::RX_RESP_GOOD);

    } else if (m_rxFrame.m_status == UWBRxStatus::TIMEOUT) {
        if (nbTries >= MAX_INTERLOC_SUBFRAMES - 1) {
            // final Rx timeout
            nbTries = 0;
            context.setState(InterlocStates::SEND_FINAL, InterlocEvent::RX_LAST_TIMEOUT);
            return;
        }

        // simple Timeout start WaitResponse again for the response of the next agent
        context.getTWR().m_responseRxTs[nbTries] = 0;
        context.setState(InterlocStates::WAIT_RESPONSE, InterlocEvent::TIMEOUT);

    } else {
        // rx in error
        while (true) {
        }
    }

    nbTries++;
}