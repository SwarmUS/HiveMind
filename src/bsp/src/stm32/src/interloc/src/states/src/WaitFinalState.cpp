#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/WaitFinalState.h>

WaitFinalState::WaitFinalState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void WaitFinalState::process(InterlocStateHandler& context) {
    volatile uint64_t finalTxTs =
        context.getTimeManager().getFinalRxTs(context.getTWR().m_pollRxTs);

    m_decawaves[DecawavePort::A].receiveDelayed(
        m_rxFrame,
        InterlocTimeManager::getTimeoutUs(context.getTimeManager().m_finalAirTimeWithPreambleUs),
        finalTxTs);

    if (m_rxFrame.m_status == UWBRxStatus::FINISHED && DecawaveUtils::isFrameFinal(m_rxFrame)) {

        context.getTWR().deserializeFinal(
            reinterpret_cast<UWBMessages::TWRFinal*>(m_rxFrame.m_rxBuffer.data()));
        context.getTWR().m_finalRxTs = m_rxFrame.m_rxTimestamp;

        context.setState(InterlocStates::SET_DISTANCE, InterlocEvent::FINAL_RECVD);

    } else if (m_rxFrame.m_status == UWBRxStatus::TIMEOUT) {
        context.setState(InterlocStates::IDLE, InterlocEvent::TIMEOUT);
    } else {
        context.setState(InterlocStates::IDLE, InterlocEvent::RX_ERROR);
    }
}