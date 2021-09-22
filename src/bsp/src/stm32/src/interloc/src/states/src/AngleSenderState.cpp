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
    do {
        sendAngleFrames(context);
    } while (InterlocBSPContainer::getInterlocManager().getState() ==
             InterlocStateDTO::ANGLE_CALIB_SENDER);

    m_decawaves.getMasterAntenna()->get().setLed(DW_LED::LED_3, false);
    context.setState(InterlocStates::IDLE, InterlocEvent::ANGLES_SENT);
}

void AngleSenderState::sendAngleFrames(const InterlocStateHandler& context) {
    for (uint32_t i = 0; i < context.getAngleNumberOfFrames() * m_transmissionContingencyFactor;
         i++) {
        m_msg.m_frameId = i;
        uint64_t sysTime = m_decawaves.getMasterAntenna()->get().getSysTime();
        m_decawaves.getMasterAntenna()->get().transmitDelayed((uint8_t*)&m_msg, sizeof(m_msg),
                                                              sysTime + (100 * UUS_TO_DWT_TIME));
    }
}
