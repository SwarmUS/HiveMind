#include <interloc/Decawave.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>
#include <states/SetDistanceState.h>

SetDistanceState::SetDistanceState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void SetDistanceState::process(InterlocStateHandler& context) {
    static bool s_ledValue;
    static uint16_t s_val = 0;
    std::optional<double> distance = context.getTWR().calculateDistance(context.getSlotId());
    if (distance) {
        m_logger.log(LogLevel::Warn, "%d", s_val++);
        //                context.getCurrentFrameId(),
        //                             distance.value());
        if (s_ledValue) {
            m_decawaves[DecawavePort::A].setLed(DW_LED::LED_2, s_ledValue);
            s_ledValue = false;
        } else {
            m_decawaves[DecawavePort::A].setLed(DW_LED::LED_2, s_ledValue);
            s_ledValue = true;
        }

        context.setState(InterlocStates::IDLE, InterlocEvent::GOOD_DISTANCE);
        //        TODO notify Buzz that a new distance value is available
    } else {
        context.setState(InterlocStates::IDLE, InterlocEvent::BAD_DISTANCE);
    }
}