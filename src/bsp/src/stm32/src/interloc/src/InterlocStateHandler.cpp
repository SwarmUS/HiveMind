#include "interloc/InterlocStateHandler.h"
#include <bsp/BSPContainer.h>

InterlocStateHandler::InterlocStateHandler() :
    m_currentState(&InterlocStateContainer::getState(InterlocStates::DEFAULT)) {}

void InterlocStateHandler::setState(InterlocStates state) {
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