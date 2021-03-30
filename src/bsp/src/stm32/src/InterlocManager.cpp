#include "InterlocManager.h"
#include "bsp/BSPContainer.h"
#include <Task.h>

InterlocManager::InterlocManager(ILogger& logger) :
    m_logger(logger), m_decaA(DW_A), m_decaB(DW_B) {}

void InterlocManager::setPositionUpdateCallback(positionUpdateCallbackFunction_t callback,
                                                void* context) {
    m_positionUpdateCallback = callback;
    m_positionUpdateCallbackContext = context;
}

void InterlocManager::startInterloc() {
    if (!m_decaA.init()) {
        m_logger.log(LogLevel::Warn, "Could not start Decawave A");
    }
    if (!m_decaB.init()) {
        m_logger.log(LogLevel::Warn, "Could not start Decawave B");
    }

    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
    UWBRxFrame rxFrame;

    while (true) {
        m_decaB.receiveAsync(rxFrame, 0);
        m_decaA.transmit(data, sizeof(data));

        while (rxFrame.m_status == UWBRxStatus::ONGOING) {
            Task::delay(1);
        }

        if (rxFrame.m_status == UWBRxStatus::FINISHED) {
            m_logger.log(LogLevel::Info, "UWB packet received on DW B");
        } else {
            m_logger.log(LogLevel::Warn, "Error while receiving UWB packet on DW B");
        }

        Task::delay(1000);
    }
}
bool InterlocManager::constructUWBHeader(uint16_t destinationId,
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
