#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/SendResponseState.h>

SendResponseState::SendResponseState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void SendResponseState::process(InterlocStateHandler& context) {
    // TODO: Set destination ID
    context.constructUWBHeader(0x01, UWBMessages::DATA, UWBMessages::TWR_RESPONSE,
                               (uint8_t*)&m_respMsg, sizeof(m_respMsg));
    m_respMsg.m_subFrameId = RESPONDER_SLOT;

    uint64_t respTxTime =
        context.getTWR().m_pollRxTs + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME);

    m_decawaves[DecawavePort::A].transmitDelayed((uint8_t*)&m_respMsg, sizeof(m_respMsg),respTxTime);

    // Get the real time at which the response was sent
    context.getTWR().m_responseTxTs =
        m_decawaves[DecawavePort::A].getTxTimestampFromDelayedTime(respTxTime);

    //transfer to wait final
    context.setState(InterlocStates::WAIT_FINAL);
    // temp - fix problem with merge
        context.getTWR().m_finalRxTs = m_rxFrame.m_rxTimestamp;
        context.getTWR().deserializeFinal(
            reinterpret_cast<UWBMessages::TWRFinal*>(m_rxFrame.m_rxBuffer.data()));
        std::optional<double> distance = context.getTWR().calculateDistance();
        if (distance) {
            m_logger.log(LogLevel::Info, "Distance: %2.3f", distance.value());
        }

        context.setState(InterlocStates::WAIT_POLL, InterlocEvent::FINAL_RECVD);
    }

    context.setState(InterlocStates::WAIT_POLL, InterlocEvent::TIMEOUT);
}

