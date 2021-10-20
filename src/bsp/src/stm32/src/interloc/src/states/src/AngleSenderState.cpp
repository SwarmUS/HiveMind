#include <Task.h>
#include <interloc/InterlocBSPContainer.h>
#include <interloc/InterlocStateHandler.h>
#include <states/AngleSenderState.h>

AngleSenderState::AngleSenderState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void AngleSenderState::process(InterlocStateHandler& context) {
    context.constructUWBHeader(UWB_BROADCAST_ADDRESS, UWBMessages::DATA, UWBMessages::ANGLE,
                               (uint8_t*)(&m_msg), sizeof(m_msg));

    m_decawaves.getMasterAntenna()->get().setLed(DW_LED::LED_3, true);

    // If we are in ANGLE_CALIB_SENDER mode, don't stop sending. Otherwise only send once
    while (InterlocBSPContainer::getInterlocManager().getState() ==
           InterlocStateDTO::ANGLE_CALIB_SENDER) {
        sendAngleFramesContinuousMode(context);
    }

    if (InterlocBSPContainer::getInterlocManager().getState() == InterlocStateDTO::OPERATING) {
        sendAngleFramesNormalMode(context);
    }

    m_decawaves.getMasterAntenna()->get().setLed(DW_LED::LED_3, false);
    context.setState(InterlocStates::IDLE, InterlocEvent::ANGLES_SENT);
}

void AngleSenderState::sendAngleFramesContinuousMode(const InterlocStateHandler& context) {
    uint64_t lastAngleTime = m_decawaves.getMasterAntenna()->get().getSysTime();

    for (uint32_t i = 0; i < NUM_ANGLE_MSG_SENDER; i++) {
        m_msg.m_messageId = i;
        lastAngleTime =
            (lastAngleTime + context.getTimeManager().getAngleToAngleOffsetUs() * UUS_TO_DWT_TIME) %
            UINT40_MAX;
        m_decawaves.getMasterAntenna()->get().transmitDelayed((uint8_t*)&m_msg, sizeof(m_msg),
                                                              lastAngleTime);
    }
}

void AngleSenderState::sendAngleFramesNormalMode(const InterlocStateHandler& context) {
    for (uint32_t i = 0; i < NUM_ANGLE_MSG_SENDER; i++) {
        m_msg.m_messageId = i;
        uint64_t angleTxTime =
            context.getTimeManager().getAngleTxStartTs(context.getPreviousFrameStartTs(), i);
        m_decawaves.getMasterAntenna()->get().transmitDelayed((uint8_t*)&m_msg, sizeof(m_msg),
                                                              angleTxTime);
    }
}
