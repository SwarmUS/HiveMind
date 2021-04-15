#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/WaitFinalState.h>

WaitFinalState::WaitFinalState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}


void WaitFinalState::process(InterlocStateHandler& context){
    m_decawaves[DecawavePort::A].receiveDelayed(m_rxFrame,FINAL_RX_TIMEOUT_UUS,0); // TODO change timeout value

    if (m_rxFrame.m_status == UWBRxStatus::FINISHED &&
        reinterpret_cast<UWBMessages::DWFrame*>(m_rxFrame.m_rxBuffer.data())->m_functionCode ==
            UWBMessages::FunctionCode::TWR_FINAL) {

        context.getTWR().m_finalRxTs = m_rxFrame.m_rxTimestamp;
        context.getTWR().deserializeFinal(
            reinterpret_cast<UWBMessages::TWRFinal*>(m_rxFrame.m_rxBuffer.data()));
        std::optional<double> distance = context.getTWR().calculateDistance();

        if (distance) {
            m_logger.log(LogLevel::Info, "Distance: %2.3f", distance.value());

            context.setState(InterlocStates::WAIT_POLL, InterlocEvent::FINAL_RECVD);
        }else{
            context.setState(InterlocStates::WAIT_POLL, InterlocEvent::DISTANCE_ERROR);
        }

    }else if(m_rxFrame.m_status == UWBRxStatus::TIMEOUT){
        context.setState(InterlocStates::WAIT_POLL, InterlocEvent::TIMEOUT);
    }else{
        context.setState(InterlocStates::WAIT_POLL, InterlocEvent::RX_ERROR);
    }

}