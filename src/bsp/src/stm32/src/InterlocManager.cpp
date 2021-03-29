#include "InterlocManager.h"
#include "bsp/BSPContainer.h"
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
//    while(true) {
//        if (m_decaB.getState() == DW_STATE::RECEIVE_CALIB) {
//            receiveTWRSequence(0x69, m_decaB);
//        }
//        if (m_decaB.getState() == DW_STATE::SEND_CALIB){
//            sendTWRSequence(0x69, m_decaB);
//        }
//        Task::delay(1000);
//    }
//    startCalibAntennaRespond(0x69, m_decaB); //1
    startCalibAntennaInit(0x69, m_decaB); //2

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

// must start with button
void InterlocManager::startCalibAntennaInit(uint16_t destinationId, Decawave& device) {
    device.setChannel(UWBChannel::CHANNEL_2);
    device.setTxAntennaDLY(DEFAULT_TX_ANT_DLY);
    device.setRxAntennaDLY(DEFAULT_RX_ANT_DLY);

    device.setState(DW_STATE::SEND_CALIB);
    //start TWR
    //set_preamble_detect?
    // reset?
    while(device.getState() == DW_STATE::SEND_CALIB) { //find exit condition
        sendTWRSequence(destinationId, device);
        Task::delay(1000);
    }

}

void InterlocManager::startCalibAntennaRespond(uint16_t destinationId, Decawave& device){
    double distanceEval;
    uint16_t distanceCm = 501; //distance between devices in calibration mode in centimeters

    device.setChannel(UWBChannel::CHANNEL_2);
    device.setTxAntennaDLY(DEFAULT_TX_ANT_DLY);
    device.setRxAntennaDLY(DEFAULT_RX_ANT_DLY);
    //set_preamble_detect?
    // reset?
    device.setState(DW_STATE::RECEIVE_CALIB);
    while(device.getState() == DW_STATE::RECEIVE_CALIB) { //find exit condition
        receiveTWRSequence(destinationId, device);
        Task::delay(100);

//        if(abs(distanceEval - distanceCm) < 10){
//            device.setState(DW_STATE::CALIBRATED);
            //send dtop calib message
//        }else {
            // set new antenna delay values
//        }

    }
}

double InterlocManager::receiveTWRSequence(uint16_t destinationId, Decawave& device) {
    UWBRxFrame rxFrame;
    UWBRxFrame secrxFrame;
    uint64_t pollRxTs;
    uint64_t respTxTs;
    UWBMessages::TWRResponse responseMsg{};
    double varA;
    double varB;
    double varC;
    double varD;
    double distanceEval;
    double tof;
    int64_t tofDtu;

    // receive poll message
    device.receive(rxFrame, 0);
    pollRxTs = rxFrame.m_rxTimestamp;

    // set timestamp for Response send
    respTxTs = (pollRxTs + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME));


    // construct and send Response message
    if(rxFrame.m_status != UWBRxStatus::FINISHED) {
        return 0;
    }

    constructUWBHeader(destinationId, UWBMessages::DATA, UWBMessages::TWR_RESPONSE,
                       (uint8_t*)&responseMsg, sizeof(responseMsg));
    responseMsg.m_calculatedTOF = 0;
//    device.transmit((uint8_t*)&responseMsg,sizeof(responseMsg));
    device.transmitDelayed((uint8_t*)&responseMsg,sizeof(responseMsg),respTxTs);

    // wait for Final message reception
    device.receive(secrxFrame, 0);
    if(secrxFrame.m_status != UWBRxStatus::FINISHED) {
        return 0;
    }
    if(secrxFrame.m_frame->m_functionCode == UWBMessages::FunctionCode::TWR_FINAL){
        m_logger.log(LogLevel::Error,"received final TWR");
    }

    return 1;
}
bool InterlocManager::sendTWRSequence(uint16_t destinationId, Decawave& device){
    UWBRxFrame responseFrame;
    UWBRxFrame finalFrame;

    UWBMessages::TWRPoll pollMsg{};
    UWBMessages::TWRFinal finalMsg{};

    uint64_t responseTimestamp;
    uint64_t pollTimestamp;
    uint64_t finalTxTime;
    uint64_t finalTxTs;

    // Construct poll message
    constructUWBHeader(destinationId,UWBMessages:: BEACON,UWBMessages::TWR_POLL,(uint8_t*)&pollMsg,sizeof(pollMsg));
    device.transmitAndReceiveDelayed((uint8_t*)(&pollMsg), sizeof(UWBMessages::DWFrame),POLL_TX_TO_RESP_RX_DLY_UUS,responseFrame,UUS_TO_DWT_TIME);

    //  Retrieve poll transmission and response reception timestamp.
    device.getTxTimestamp(&pollTimestamp);;
    responseTimestamp = responseFrame.m_rxTimestamp;

    //  Compute final message transmission time
    finalTxTime = (responseTimestamp + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME));
    finalTxTs = ((uint64_t)(finalTxTime & 0xFFFFFFFEUL)) + device.getTxAntennaDLY();

    //  Construct final message
    finalMsg.m_respMinPoll = responseTimestamp - pollTimestamp;
    finalMsg.m_finaleMinResp = finalTxTs - responseTimestamp;

    constructUWBHeader(destinationId,UWBMessages::DATA,UWBMessages::TWR_FINAL,(uint8_t*)(&finalMsg),sizeof(finalMsg));
    return device.transmit((uint8_t*)(&finalMsg),sizeof(UWBMessages::TWRFinal));
}