#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/WaitPollState.h>

WaitPollState::WaitPollState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void WaitPollState::process(InterlocStateHandler& context) {

    uint64_t rxStartTs = context.getTimeManager().getPollRxStartTs(
        context.getPreviousFrameStartTs() + 140 * UUS_TO_DWT_TIME);

    if (context.getCurrentFrameId() == context.getSlotId() + 1) { // was the previous frame leader
        volatile uint64_t currentTime = m_decawaves[DecawavePort::A].getSysTime();
        (void)currentTime;
    }
    m_decawaves[DecawavePort::A].receiveDelayed(
        m_rxFrame,
        InterlocTimeManager::getTimeoutUs(context.getTimeManager().m_pollAirTimeWithPreambleUs),
        rxStartTs);

    if (m_rxFrame.m_status == UWBRxStatus::FINISHED && DecawaveUtils::isFramePoll(m_rxFrame)) {

        UWBMessages::TWRPoll* msg =
            reinterpret_cast<UWBMessages::TWRPoll*>(m_rxFrame.m_rxBuffer.data());

        context.getTWR().m_pollRxTs = m_rxFrame.m_rxTimestamp;
        context.setPreviousFrameStartTs(m_rxFrame.m_rxTimestamp);
        context.setSuperFrameInitiator(msg->m_superFrameInitiator);
        context.setCurrentFrameId(msg->m_currentFrameId);

        //        context.getTWR().m_pollRxTs = m_rxFrame.m_rxTimestamp;

        // sychronisation parameters

        //        context.setPreviousFrameStartTs(m_rxFrame.m_rxTimestamp);
        //        context.setSuperFrameInitiator(
        //        reinterpret_cast<UWBMessages::TWRPoll*>(m_rxFrame.m_rxBuffer.data())
        //                ->m_superFrameInitiator);
        //        context.setCurrentFrameId(
        //            reinterpret_cast<UWBMessages::TWRPoll*>(m_rxFrame.m_rxBuffer.data())->m_currentFrameId);

        context.setState(InterlocStates::SEND_RESPONSE, InterlocEvent::POLL_RECVD);
        m_logger.log(LogLevel::Warn, "p %d", context.getCurrentFrameId());
        return;
    }
    //    volatile uint64_t currentTime = m_decawaves[DecawavePort::A].getSysTime();
    //    (void)currentTime;
    context.setPreviousFrameStartTs(rxStartTs +
                                    (RX_BEFORE_TX_GUARD_US +
                                     (uint64_t)context.getTimeManager().getPreambleAirTimeUs() +
                                     POLL_PROCESSING_GUARD) *
                                        UUS_TO_DWT_TIME);
    context.setState(InterlocStates::IDLE, InterlocEvent::TIMEOUT);
}
