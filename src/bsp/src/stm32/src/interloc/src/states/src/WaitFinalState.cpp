#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/WaitFinalState.h>

WaitFinalState::WaitFinalState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void WaitFinalState::process(InterlocStateHandler& context) {
    m_decawaves[DecawavePort::A].receive(m_rxFrame, InterlocTimeManager::getFinalTimeout());

    if (m_rxFrame.m_status == UWBRxStatus::FINISHED && DecawaveUtils::isFrameFinal(m_rxFrame)) {

        context.getTWR().deserializeFinal(
            reinterpret_cast<UWBMessages::TWRFinal*>(m_rxFrame.m_rxBuffer.data()));
        context.getTWR().m_finalRxTs = m_rxFrame.m_rxTimestamp;

        std::optional<double> distance = context.getTWR().calculateDistance(context.getSlotId());

        if (distance) {
            m_logger.log(LogLevel::Info, "Distance: %2.3f", distance.value());

            context.setState(InterlocStates::IDLE, InterlocEvent::FINAL_RECVD);
        } else {
            context.setState(InterlocStates::IDLE, InterlocEvent::DISTANCE_ERROR);
        }

    } else if (m_rxFrame.m_status == UWBRxStatus::TIMEOUT) {
        context.setState(InterlocStates::IDLE, InterlocEvent::TIMEOUT);
    } else {
        context.setState(InterlocStates::IDLE, InterlocEvent::RX_ERROR);
    }
}