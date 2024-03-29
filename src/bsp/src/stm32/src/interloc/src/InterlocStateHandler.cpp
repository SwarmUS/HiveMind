#include "interloc/InterlocStateHandler.h"
#include <bsp/BSPContainer.h>

InterlocStateHandler::InterlocStateHandler(InterlocTimeManager& timeManager,
                                           BspInterlocRawAngleData& angleRawData) :
    m_timeManager(timeManager),
    m_currentStateName(InterlocStates::DEFAULT),
    m_currentState(&InterlocStateContainer::getState(InterlocStates::DEFAULT)),
    m_angleRawData(angleRawData),
    m_stateTracer(m_stateTracerData.data(), MAX_TRACER_TRANSITIONS) {

    m_slotId = getSlotIdFromBoardId(BSPContainer::getBSP().getUUId());

    m_timeManager.setNumSlots(m_numFrames);
    m_timeManager.setSlodId(m_slotId);
}

void InterlocStateHandler::setState(InterlocStates state, InterlocEvent event) {
    if (m_stateTracer.isFull()) {
        m_stateTracer.pop();
    }

    StateTransition stateTransition{m_currentStateName, state, event, m_currentTwrFrame};
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

void InterlocStateHandler::incrementCurrentTwrFrame() {
    m_currentTwrFrame++;

    if (m_currentTwrFrame > m_numFrames) {
        m_currentTwrFrame = 1;
    }
}

InterlocTimeManager& InterlocStateHandler::getTimeManager() const { return m_timeManager; }

uint8_t InterlocStateHandler::getSlotId() const { return m_slotId; }
uint8_t InterlocStateHandler::getSuperFrameInitiator() const { return m_superFrameInitiator; }
uint8_t InterlocStateHandler::getNumFrames() const { return m_numFrames; }
uint8_t InterlocStateHandler::getCurrentFrameId() const { return m_currentTwrFrame; }

void InterlocStateHandler::setSuperFrameInitiator(uint8_t initiatorId) {
    m_superFrameInitiator = initiatorId;
}
void InterlocStateHandler::setCurrentFrameId(uint8_t frameId) { m_currentTwrFrame = frameId; }
uint64_t InterlocStateHandler::getPreviousFrameStartTs() const { return m_previousFrameStartTs; }
void InterlocStateHandler::setPreviousFrameStartTs(uint64_t timestamp) {
    m_previousFrameStartTs = timestamp;
}

uint16_t InterlocStateHandler::getSlotIdFromBoardId(uint16_t boardId) { return boardId; }
uint16_t InterlocStateHandler::getBoardIdFromSlotId(uint16_t slotId) { return slotId; }

void InterlocStateHandler::setAngleCalibNumberOfFrames(uint32_t numberOfFrames) {
    m_angleCalibNumberOfFrames = numberOfFrames;
}

uint32_t InterlocStateHandler::getAngleCalibNumberOfFrames() const {
    return m_angleCalibNumberOfFrames;
}
BspInterlocRawAngleData& InterlocStateHandler::getRawAngleData() { return m_angleRawData; }
