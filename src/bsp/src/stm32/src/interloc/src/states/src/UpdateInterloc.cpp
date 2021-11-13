#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/UpdateInterloc.h>

UpdateInterloc::UpdateInterloc(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void UpdateInterloc::process(InterlocStateHandler& context) {
    std::optional<double> distance = context.getTWR().calculateDistance(context.getSlotId());
    std::optional<float> angle = {};
    std::optional<float> losConfidence = {};
    std::optional<bool> los = {};

    if (m_decawaves.canCalculateAngles()) {
        auto ret =
            InterlocBSPContainer::getAngleCalculator().calculateAngle(context.getRawAngleData());

        angle = std::get<0>(ret);
        losConfidence = std::get<1>(ret);
    }

    // Reset data for next run
    context.getTWR().m_allDataReceived = false;
    context.getRawAngleData().m_framesLength = 0;

    if (distance && angle) {
        m_logger.log(LogLevel::Debug, "Interloc Calculated with #%d. Dist: %3.3f, Angle: %3.3f",
                     context.getCurrentFrameId(), distance.value(), angle.value());
    } else if (distance) {
        m_logger.log(LogLevel::Debug, "Interloc Calculated with #%d. Dist: %3.3f, Angle: err",
                     context.getCurrentFrameId(), distance.value());
    } else if (angle) {
        m_logger.log(LogLevel::Debug, "Interloc Calculated with #%d. Dist: err, Angle: %3.3f",
                     context.getCurrentFrameId(), angle.value());
    }

    if (losConfidence) {
        if (losConfidence > 0.5) {
            los = true;
        } else {
            los = false;
        }
    }

    // Update using the optionals. The higher levels take care of updating only if a value is
    // present.
    InterlocBSPContainer::getInterlocManager().updateInterloc(
        InterlocStateHandler::getBoardIdFromSlotId(context.getCurrentFrameId()), distance, angle,
        los);

    context.setState(InterlocStates::IDLE, InterlocEvent::NO_EVENT);
}
