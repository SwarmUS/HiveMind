#include "InterlocManager.h"
#include <Task.h>

InterlocManager::InterlocManager(ILogger& logger) :
    m_logger(logger), m_decaA(DW_A), m_decaB(DW_B) {}

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
// must start with button
void InterlocManager::startCalibAntenna(Decawave device, uint16_t distance) {
    device.setChannel(UWBChannel::CHANNEL_5);
    device.setTxAntennaDLY(TX_ANT_DLY);
    device.setRxAntennaDLY(RX_ANT_DLY);

    uint8_t erDLY = 0;
    uint8_t error_margin = 0;
    UWBRxFrame frame;
    //start TWR
    while(erDLY > error_margin ){
        device.transmitAndReceiveDelayed(twrMSg, 0,POLL_TX_TO_RESP_RX_DLY_UUS,frame,RESP_RX_TIMEOUT_UUS);
    }
    //for 1000 times
}

//void sentTo(Id, mes);