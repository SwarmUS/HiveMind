#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/WaitResponseState.h>

WaitResponseState::WaitResponseState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void WaitResponseState::process(InterlocStateHandler& context) {
    // begin to receive for a small time window corresponding to the TWRResponse Rx slot on the
    // remote agents
    m_decawaves[DecawavePort::A].receiveDelayed(
        m_rxFrame,
        InterlocTimeManager::getTimeoutUs(context.getTimeManager().m_responseAirTimeWithPreambleUs),
        context.getTimeManager().m_responseRxTs[nbTries]);

    if (m_rxFrame.m_status == UWBRxStatus::FINISHED && DecawaveUtils::isFrameResponse(m_rxFrame)) {

        m_decawaves[DecawavePort::A].getTxTimestamp(&context.getTWR().m_pollTxTs);
        uint8_t index =
            reinterpret_cast<UWBMessages::TWRResponse*>(m_rxFrame.m_rxBuffer.data())->m_subFrameId -
            1;
        context.getTWR().m_responseRxTs[index] = m_rxFrame.m_rxTimestamp;
        nbRespReceived++;
        if (index == MAX_INTERLOC_SUBFRAMES - 1) {
            // Last Response, send Final to all
            nbRespReceived = 0;
            nbTries = 0;
            context.setState(InterlocStates::SEND_FINAL, InterlocEvent::RX_LAST_RESP);
            return;
        }
        // this was not the last expected Rx, continue to receive
        context.setState(InterlocStates::WAIT_RESPONSE, InterlocEvent::RX_RESP_GOOD);

    } else if (m_rxFrame.m_status == UWBRxStatus::TIMEOUT) {
        // a timeout takes about 205us to return
        if (nbTries >= MAX_INTERLOC_SUBFRAMES - 1) {
            if (nbRespReceived > 0) {
                // final Rx timeout
                nbRespReceived = 0;
                nbTries = 0;
                context.setState(InterlocStates::SEND_FINAL, InterlocEvent::RX_LAST_TIMEOUT);
                return;
            }
            nbTries = 0;
            context.setState(InterlocStates::SEND_FINAL, InterlocEvent::RX_NO_RESP);
            m_logger.log(LogLevel::Warn, "no resp : %d", context.getCurrentFrameId());
            return;
        }

        // simple Timeout start WaitResponse again for the response of the next agents
        context.getTWR().m_responseRxTs[nbTries] = 0;
        context.setState(InterlocStates::WAIT_RESPONSE, InterlocEvent::TIMEOUT);

    } else {
        // rx in error. For debugging purposes
        //         while (true) {
        //         }
    }

    nbTries++;
}