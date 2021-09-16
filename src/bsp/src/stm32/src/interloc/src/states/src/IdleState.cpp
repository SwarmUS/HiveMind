#include "states/IdleState.h"
#include <Task.h>
#include <interloc/InterlocBSPContainer.h>
#include <interloc/InterlocStateHandler.h>

IdleState::IdleState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void IdleState::process(InterlocStateHandler& context) {
    while (true) {
        InterlocStateDTO managerState = InterlocBSPContainer::getInterlocManager().getState();

        switch (managerState) {
        case InterlocStateDTO::OPERATING:
            processOperatingMode(context);
            return;

        case InterlocStateDTO::ANGLE_CALIB_SENDER:
            context.setState(InterlocStates::ANGLE_SENDER, InterlocEvent::API_MESSAGE);
            return;

        case InterlocStateDTO::ANGLE_CALIB_RECEIVER:
            context.setState(InterlocStates::ANGLE_RECEIVER, InterlocEvent::API_MESSAGE);
            return;

        default:
            // Wait and recheck state
            Task::delay(10);
        }
    }
}

void IdleState::processOperatingMode(InterlocStateHandler& context) {
    // First time we pass by the Idle state
    if (context.getSuperFrameInitiator() == 0) {
        context.setState(InterlocStates::SYNC, InterlocEvent::BOOTUP);
        return;
    }

    context.incrementCurrentFrameId();
    uint8_t currentFrameId = context.getCurrentFrameId();

    if (currentFrameId == (context.getSuperFrameInitiator())) {
        context.setState(InterlocStates::SYNC, InterlocEvent::SUPERFRAME_FINISHED);
        return;
    }

    if (currentFrameId == context.getSlotId()) {
        context.setState(InterlocStates::SEND_POLL, InterlocEvent::NEXT_FRAME_LEADER);
        return;
    }

    context.setState(InterlocStates::WAIT_POLL, InterlocEvent::NOT_NEXT_FRAME_LEADER);
}