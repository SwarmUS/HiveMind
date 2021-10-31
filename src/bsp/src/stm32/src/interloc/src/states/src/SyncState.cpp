#include "states/SyncState.h"
#include <interloc/DecawaveUtils.h>
#include <interloc/InterlocStateHandler.h>

#define RX_RELOAD_OVERHEAD_US 100

SyncState::SyncState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void SyncState::process(InterlocStateHandler& context) {
    // keep around for speed debug
    //    m_logger.log(LogLevel::Warn, "+1");

    if (auto deca = m_decawaves.getMasterAntenna()) {
        uint64_t syncStartTs = deca->get().getSysTime();
        uint32_t initialRxTimeoutUs = context.getTimeManager().getSyncTimeoutUs();
        uint32_t remainingRxTimeoutUs = initialRxTimeoutUs;
        uint16_t rxTimeoutUs = 0;

        capTimeoutUint16(remainingRxTimeoutUs, rxTimeoutUs);

        while (rxTimeoutUs > RX_RELOAD_OVERHEAD_US) {
            UWBRxStatus status = deca->get().receive(rxTimeoutUs);

            if (status == UWBRxStatus::TIMEOUT && remainingRxTimeoutUs > 0) {
                capTimeoutUint16(remainingRxTimeoutUs, rxTimeoutUs);
            } else {
                if (status == UWBRxStatus::TIMEOUT) {
                    context.setState(InterlocStates::SEND_POLL_FROM_SYNC, InterlocEvent::TIMEOUT);
                    return;
                }

                deca->get().retrieveRxFrame(m_rxFrame);
                if (DecawaveUtils::isFramePoll(m_rxFrame)) {
                    return handlePollReceived(context);
                }

                // If an unsupported message is received, restart RX for the remainder of the
                // timeout
                remainingRxTimeoutUs = initialRxTimeoutUs -
                                       ((deca->get().getSysTime() - syncStartTs) / UUS_TO_DWT_TIME);
                capTimeoutUint16(remainingRxTimeoutUs, rxTimeoutUs);

                // Because of the subtraction, there could be an underflow.
                if (rxTimeoutUs > initialRxTimeoutUs) {
                    rxTimeoutUs = 0;
                }
            }
        }

        context.setState(InterlocStates::SYNC, InterlocEvent::RX_ERROR);
    } else {
        context.setState(InterlocStates::IDLE, InterlocEvent::DECA_INIT_ERROR);
    }
}
void SyncState::capTimeoutUint16(uint32_t& remainingRxTimeoutUs, uint16_t& rxTimeoutUs) const {
    if (remainingRxTimeoutUs > UINT16_MAX) {
        rxTimeoutUs = UINT16_MAX;
        remainingRxTimeoutUs = remainingRxTimeoutUs - UINT16_MAX;
    } else {
        rxTimeoutUs = remainingRxTimeoutUs;
        remainingRxTimeoutUs = 0;
    }
}

void SyncState::handlePollReceived(InterlocStateHandler& context) {
    UWBMessages::TWRPoll* msg =
        reinterpret_cast<UWBMessages::TWRPoll*>(m_rxFrame.m_rxBuffer.data());

    context.getTWR().m_pollRxTs = m_rxFrame.m_rxTimestamp;
    context.setPreviousFrameStartTs(m_rxFrame.m_rxTimestamp);
    context.setSuperFrameInitiator(msg->m_superFrameInitiator);
    context.setCurrentFrameId(msg->m_currentFrameId);

    context.setState(InterlocStates::SEND_RESPONSE, InterlocEvent::POLL_RECVD);
}