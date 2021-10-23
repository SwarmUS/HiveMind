#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <math.h>
#include <states/InterlocStateContainer.h>
#include <states/UpdateInterloc.h>

UpdateInterloc::UpdateInterloc(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {

    for (unsigned int i = 0; i < 5; i++) {
        pdoa_Origin[i] = (-90 - pdoa_B[i]) / pdoa_M;
    }
}

void UpdateInterloc::process(InterlocStateHandler& context) {
    std::optional<double> distance = context.getTWR().calculateDistance(context.getSlotId());
    if (distance) {
        m_logger.log(LogLevel::Warn, "distance from : %d = %3.3f", context.getCurrentFrameId(),
                     distance.value());

        InterlocBSPContainer::getInterlocManager().updateDistance(
            InterlocStateHandler::getBoardIdFromSlotId(context.getCurrentFrameId()),
            distance.value());

        context.setState(InterlocStates::IDLE, InterlocEvent::GOOD_DISTANCE);
    } else {
        // TODO : what happens on failure?
        m_logger.log(LogLevel::Warn, "distance from : %d = Err", context.getCurrentFrameId());
        context.setState(InterlocStates::IDLE, InterlocEvent::BAD_DISTANCE);
    }

    float angle = 0;
    if (context.getRawAngleData().m_framesLength >= 30) {
        angle = calculateAngle(context);
    }
    m_logger.log(LogLevel::Warn, "Angle from : %d = %3.2f", context.getCurrentFrameId(), angle);
    // TODO: Add angle calculation
}

float UpdateInterloc::calculateAngle(InterlocStateHandler& context) {
    float angleAccum = 0;

    for (unsigned int i = 0; i < 30; i++) {
        volatile int64_t tdoa =
            (int64_t)(context.getRawAngleData().m_frames[i].m_frameInfos[0].m_rxTimestamp -
                      context.getRawAngleData().m_frames[i].m_frameInfos[2].m_rxTimestamp);

        volatile float tdoaDist = (float)tdoa / UUS_TO_DWT_TIME * 299.792458;

        tdoaDist /= 0.36;
        angleAccum += asin(tdoaDist) * 180 / M_PI;
    }

    float angle = angleAccum / 30;
    float angleLinearized = (angle - 74.16) / -0.41;

    volatile unsigned int selectedPdoa = 0;

    for (unsigned int i = 0; i < 5; i++) {
        if (angleLinearized > pdoa_Origin[i]) {
            selectedPdoa = i;
        }
    }

    volatile float phaseDifference =
        context.getRawAngleData().m_frames[15].m_frameInfos[0].m_accumulatorAngle -
        context.getRawAngleData().m_frames[15].m_frameInfos[0].m_sfdAngle -
        context.getRawAngleData().m_frames[15].m_frameInfos[2].m_accumulatorAngle +
        context.getRawAngleData().m_frames[15].m_frameInfos[2].m_sfdAngle + M_PI;

    while (phaseDifference > (2 * M_PI)) {
        phaseDifference -= (2 * M_PI);
    }

    phaseDifference -= M_PI;
    volatile float anglePdoa = asin(phaseDifference / M_PI) * (1 / 0.75) * 180 / M_PI;

    volatile float calculatedPdoa = (anglePdoa - pdoa_B[selectedPdoa]) / pdoa_M;

    return calculatedPdoa;
}
