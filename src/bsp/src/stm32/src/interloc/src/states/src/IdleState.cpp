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
        volatile uint64_t currentTime = m_decawaves[DecawavePort::A].getSysTime();
        (void)currentTime;
        context.setState(InterlocStates::SYNC, InterlocEvent::SUPERFRAME_FINISHED);
        return;
    }

    if (currentFrameId == context.getSlotId()) {
        context.setState(InterlocStates::SEND_POLL, InterlocEvent::NEXT_FRAME_LEADER);
        return;
    }

    context.setState(InterlocStates::WAIT_POLL, InterlocEvent::NOT_NEXT_FRAME_LEADER);
}
