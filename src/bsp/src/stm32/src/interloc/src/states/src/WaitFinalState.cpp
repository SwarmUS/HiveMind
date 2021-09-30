#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/WaitFinalState.h>

WaitFinalState::WaitFinalState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void WaitFinalState::process(InterlocStateHandler& context) {
    if (auto deca = m_decawaves.getMasterAntenna()) {
        UWBRxStatus status = deca->get().receiveDelayed(
            InterlocTimeManager::getTimeoutUs(
                context.getTimeManager().m_finalAirTimeWithPreambleUs),
            context.getTimeManager().getFinalRxTs(context.getTWR().m_pollRxTs));

        if (status == UWBRxStatus::FINISHED) {
            deca->get().retrieveRxFrame(m_rxFrame);

            if (DecawaveUtils::isFrameFinal(m_rxFrame)) {
                context.getTWR().deserializeFinal(
                    reinterpret_cast<UWBMessages::TWRFinal*>(m_rxFrame.m_rxBuffer.data()));
                context.getTWR().m_finalRxTs = m_rxFrame.m_rxTimestamp;

                context.setState(InterlocStates::CALCULATE_INTERLOC, InterlocEvent::FINAL_RECVD);
            }
        } else if (status == UWBRxStatus::TIMEOUT) {
            context.setState(InterlocStates::IDLE, InterlocEvent::TIMEOUT);
        } else {
            context.setState(InterlocStates::IDLE, InterlocEvent::RX_ERROR);
        }
    } else {
        context.setState(InterlocStates::IDLE, InterlocEvent::DECA_INIT_ERROR);
    }
}