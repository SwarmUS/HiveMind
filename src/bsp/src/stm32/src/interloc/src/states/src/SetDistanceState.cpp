#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/SetDistanceState.h>

SetDistanceState::SetDistanceState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void SetDistanceState::process(InterlocStateHandler& context) {

    std::optional<double> distance = context.getTWR().calculateDistance(context.getSlotId());
    if (distance) {
        m_logger.log(LogLevel::Warn, "distance from : %d = %3.3f", context.getCurrentFrameId(),
                     distance.value());
        context.setState(InterlocStates::IDLE, InterlocEvent::GOOD_DISTANCE);
        //        TODO notify Buzz that a new distance value is available
    } else {
        context.setState(InterlocStates::IDLE, InterlocEvent::BAD_DISTANCE);
    }
}