#include <interloc/InterlocBSPContainer.h>
#include <interloc/InterlocStateHandler.h>
#include <states/AngleSenderState.h>

AngleSenderState::AngleSenderState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void AngleSenderState::process(InterlocStateHandler& context) {
    context.constructUWBHeader(UWB_BROADCAST_ADDRESS, UWBMessages::DATA, UWBMessages::ANGLE,
                               (uint8_t*)(&m_msg), sizeof(m_msg));

    // If we are in ANGLE_CALIB_RECEIVER mode, don't stop sending. Otherwise only send once
    do {
        sendAngleFrames(context);
    } while (InterlocBSPContainer::getInterlocManager().getState() ==
             InterlocStateDTO::ANGLE_CALIB_RECEIVER);

    context.setState(InterlocStates::IDLE, InterlocEvent::ANGLES_SENT);
}

void AngleSenderState::sendAngleFrames(const InterlocStateHandler& context) {
    for (uint32_t i = 0; i < context.getAngleNumberOfFrames() * m_transmissionContingencyFactor;
         i++) {
        m_msg.m_frameId = i;
        m_decawaves[DecawavePort::A].transmit((uint8_t*)&m_msg, sizeof(m_msg));
    }
}
