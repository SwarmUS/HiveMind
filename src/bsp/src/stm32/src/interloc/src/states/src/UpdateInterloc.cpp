#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/UpdateInterloc.h>

UpdateInterloc::UpdateInterloc(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void UpdateInterloc::process(InterlocStateHandler& context) {
    std::optional<double> distance = context.getTWR().calculateDistance(context.getSlotId());
    std::optional<float> angle =
        InterlocBSPContainer::getAngleCalculator().calculateAngle(context.getRawAngleData());

    // Reset data for next run
    context.getTWR().m_allDataReceived = false;
    context.getRawAngleData().m_framesLength = 0;

    // TODO: DEBUGGING:
    if (distance) {
        m_logger.log(LogLevel::Warn, "distance from : %d = %3.3f", context.getCurrentFrameId(),
                     distance.value());
    } else {
        m_logger.log(LogLevel::Warn, "distance from : %d = Err", context.getCurrentFrameId());
    }

    if (angle) {
        m_logger.log(LogLevel::Warn, "angle from : %d = %3.3f", context.getCurrentFrameId(),
                     distance.value());
    } else {
        m_logger.log(LogLevel::Warn, "angle from : %d = Err", context.getCurrentFrameId());
    }

    // Update using the optionals. The higher levels take care of updating only if a value is
    // present.
    InterlocBSPContainer::getInterlocManager().updateInterloc(
        InterlocStateHandler::getBoardIdFromSlotId(context.getCurrentFrameId()), distance, angle);

    context.setState(InterlocStates::IDLE, InterlocEvent::NO_EVENT);
}
