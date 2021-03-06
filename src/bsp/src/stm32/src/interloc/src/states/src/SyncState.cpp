#include "states/SyncState.h"
#include <interloc/DecawaveUtils.h>
#include <interloc/InterlocStateHandler.h>

#define RX_RELOAD_OVERHEAD_US 100

SyncState::SyncState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void SyncState::process(InterlocStateHandler& context) {
    // keep around for speed debug
    //    m_logger.log(LogLevel::Warn, "+1");

    uint64_t syncStartTs = m_decawaves[DecawavePort::A].getSysTime();
    uint32_t initialRxTimeoutUs = context.getTimeManager().getSyncTimeoutUs();
    uint32_t rxTimeoutUs = initialRxTimeoutUs;

    while (rxTimeoutUs > RX_RELOAD_OVERHEAD_US) {
        m_decawaves[DecawavePort::A].receive(m_rxFrame, rxTimeoutUs);

        if (m_rxFrame.m_status == UWBRxStatus::TIMEOUT) {
            context.setState(InterlocStates::SEND_POLL_FROM_SYNC, InterlocEvent::TIMEOUT);
            return;
        }

        if (DecawaveUtils::isFramePoll(m_rxFrame)) {
            return handlePollReceived(context);
        }

        // If an unsupported message is received, restart RX for the remainder of the timeout
        rxTimeoutUs =
            initialRxTimeoutUs - (m_decawaves[DecawavePort::A].getSysTime() - syncStartTs);

        // Because of the subtraction, there could be an underflow.
        if (rxTimeoutUs > initialRxTimeoutUs) {
            rxTimeoutUs = 0;
        }
    }

    context.setState(InterlocStates::SYNC, InterlocEvent::RX_ERROR);
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