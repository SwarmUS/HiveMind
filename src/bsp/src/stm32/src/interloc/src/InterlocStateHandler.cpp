#include "interloc/InterlocStateHandler.h"
#include <bsp/BSPContainer.h>

InterlocStateHandler::InterlocStateHandler(InterlocTimeManager& timeManager) :
    m_timeManager(timeManager),
    m_currentStateName(InterlocStates::DEFAULT),
    m_currentState(&InterlocStateContainer::getState(InterlocStates::DEFAULT)),
    m_stateTracer(m_stateTracerData.data(), MAX_TRACER_TRANSITIONS) {
    m_timeManager.setNumSlots(1);
    m_timeManager.setSlodId(m_slotId);
}

void InterlocStateHandler::setState(InterlocStates state, InterlocEvent event) {
    if (m_stateTracer.isFull()) {
        m_stateTracer.pop();
    }

    StateTransition stateTransition{m_currentStateName, state, event};
    m_stateTracer.push(stateTransition);

    m_currentStateName = state;
    m_currentState = &InterlocStateContainer::getState(state);
}

void InterlocStateHandler::process() { m_currentState->process(*this); }

TwoWayRanging& InterlocStateHandler::getTWR() { return m_twr; }

bool InterlocStateHandler::constructUWBHeader(uint16_t destinationId,
                                              UWBMessages::FrameType frameType,
                                              UWBMessages::FunctionCode functionCode,
                                              uint8_t* buffer,
                                              uint16_t bufferLength) {
    if (bufferLength < sizeof(UWBMessages::DWFrame)) {
        return false;
    }

    UWBMessages::DWFrame* header = (UWBMessages::DWFrame*)buffer;
    header->m_header = {{frameType, // Frame Type
                         0, // Security disabled
                         0, // Frame pending
                         0, // ACK request TODO: Maybe make this configurable later on
                         1, // Compress PAN ID
                         0, // Reserved
                         UWBMessages::AddressMode::SHORT_ADDRESS,
                         UWBMessages::FrameVersion::VERSION_0,
                         UWBMessages::AddressMode::SHORT_ADDRESS},
                        m_sequenceID++,
                        PAN_ID,
                        destinationId,
                        BSPContainer::getBSP().getUUId()};
    header->m_functionCode = functionCode;

    return true;
}

void InterlocStateHandler::incrementCurrentFrameId() {
    m_currentFrameId++;

    if (m_currentFrameId > m_numFrames) {
        m_currentFrameId = 1;
    }
}

InterlocTimeManager& InterlocStateHandler::getTimeManager() { return m_timeManager; }

uint8_t InterlocStateHandler::getSlotId() const { return m_slotId; }
uint8_t InterlocStateHandler::getSuperFrameInitiator() const { return m_superFrameInitiator; }
uint8_t InterlocStateHandler::getNumFrames() const { return m_numFrames; }
uint8_t InterlocStateHandler::getCurrentFrameId() const { return m_currentFrameId; }

void InterlocStateHandler::setSuperFrameInitiator(uint8_t initiatorId) {
    m_superFrameInitiator = initiatorId;
}
void InterlocStateHandler::setCurrentFrameId(uint8_t frameId) { m_currentFrameId = frameId; }