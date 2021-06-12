#include "states/IdleState.h"
#include <interloc/InterlocStateHandler.h>

IdleState::IdleState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void IdleState::process(InterlocStateHandler& context) {
    // First time we pass by the Idle state
    if (context.getSuperFrameInitiator() == 0) {
        context.setState(InterlocStates::SYNC, InterlocEvent::BOOTUP);
        return;
    }

    context.incrementCurrentFrameId();
    uint8_t currentFrameId = context.getCurrentFrameId();

    if (currentFrameId == (context.getSuperFrameInitiator())) {
        // temp wait for closer sync
        volatile uint64_t currentTime = m_decawaves[DecawavePort::A].getSysTime();
        (void)currentTime;
        //        uint64_t delayDWT =
        //            context.getTimeManager().getPollTxStartTs(context.getPreviousFrameStartTs()) -
        //            (currentTime - context.getPreviousFrameStartTs()) - 400U * UUS_TO_DWT_TIME;
        //        UWBRxFrame m_rxFrame;
        //        m_decawaves[DecawavePort::A].receive(m_rxFrame, delayDWT / UUS_TO_DWT_TIME);
        context.setState(InterlocStates::SYNC, InterlocEvent::SUPERFRAME_FINISHED);

        //        context.setState(InterlocStates::WAIT_POLL, InterlocEvent::NOT_NEXT_FRAME_LEADER);
        return;
    }

    if (currentFrameId == context.getSlotId()) {
        context.setState(InterlocStates::SEND_POLL, InterlocEvent::NEXT_FRAME_LEADER);
        return;
    }

    // TEMP
    // if not next_frame_leader
    context.setState(InterlocStates::WAIT_POLL, InterlocEvent::NOT_NEXT_FRAME_LEADER);
    //    context.setState(InterlocStates::SYNC, InterlocEvent::NOT_NEXT_FRAME_LEADER);
    //    context.setState(InterlocStates::SEND_POLL, InterlocEvent::NOT_NEXT_FRAME_LEADER);
}
