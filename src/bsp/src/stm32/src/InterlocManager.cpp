#include "InterlocManager.h"
#include "bsp/BSPContainer.h"
#include <Task.h>

bool isFrameGood(UWBRxFrame frame,ILogger&  logger){

    if(frame.m_status == UWBRxStatus::FINISHED) {
        //logger.log(LogLevel::Error,"Good message");
    }else if(frame.m_status == UWBRxStatus::ERROR){
        logger.log(LogLevel::Error,"Message in error");
        return false;
    }else{
        logger.log(LogLevel::Error,"Message Timeout");
        return false;
    }
    return true;
}

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
    startCalibAntennaRespond(0x69, m_decaB); //1
//    startCalibAntennaInit(0x69, m_decaB); //2

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
    volatile uint64_t pollRxTs;
    uint64_t sysTs;
    uint32_t respTxTime;
    uint64_t respTxTs;
    uint64_t finalRxTs;
    UWBMessages::TWRResponse responseMsg{};
    volatile double varA;
    volatile double varB;
    volatile double varC;
    volatile double varD;
    volatile double distanceEval;
    double tof;
    uint64_t tofDtu;

    // receive poll message
    device.receive(rxFrame, 0);
    pollRxTs = rxFrame.m_rxTimestamp;

    device.getSysTime(&sysTs);
    (void) sysTs;
    // set timestamp for Response send
    respTxTime = (pollRxTs + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
//    respTxTime = (pollRxTs + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME));
    (void) respTxTime;

    // construct and send Response message
    if(!isFrameGood(rxFrame, m_logger)){
        return 0;
    }
    if(rxFrame.m_frame->m_functionCode == UWBMessages::FunctionCode::TWR_POLL){
      //  m_logger.log(LogLevel::Error,"received POLL TWR");
    }else{
        return 0;
    }

    constructUWBHeader(destinationId, UWBMessages::DATA, UWBMessages::TWR_RESPONSE,
                       (uint8_t*)&responseMsg, sizeof(responseMsg));
    responseMsg.m_calculatedTOF = 0;
    device.transmit((uint8_t*)&responseMsg,sizeof(responseMsg));
//    device.transmitDelayed((uint8_t*)&responseMsg,sizeof(responseMsg),respTxTime);

    // wait for Final message reception
    device.receive(rxFrame, 0);
    if(!isFrameGood(rxFrame, m_logger)){
        return 0;
    }
    if(rxFrame.m_frame->m_functionCode == UWBMessages::FunctionCode::TWR_FINAL){
      //  m_logger.log(LogLevel::Error,"received final TWR");
    }else{
        m_logger.log(LogLevel::Error,"received Not final TWR");
        return 0;
    }

    UWBMessages::TWRFinal* finalTWRFrame = (UWBMessages::TWRFinal*)(rxFrame.m_frame);

    device.getTxTimestamp(&respTxTs);
    finalRxTs = rxFrame.m_rxTimestamp;

    // evaluate distance
    varA = (double)finalTWRFrame->m_respMinPoll;
    varB = (double)((uint32_t)finalRxTs-(uint32_t)respTxTs);
    varC = (double)(finalTWRFrame->m_finaleMinResp);
    varD = (double)((uint32_t)respTxTs-(uint32_t)pollRxTs);

    tofDtu = ((varA * varB - varC * varD) / (varA + varB + varC + varD));

    tof = tofDtu * DWT_TIME_UNITS;
    distanceEval = tof * SPEED_OF_LIGHT;
    (void) distanceEval;
//    return distanceEval;
    m_logger.log(LogLevel::Error,"distance : %f", distanceEval);
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
    device.transmitAndReceiveDelayed((uint8_t*)(&pollMsg), sizeof(UWBMessages::DWFrame),POLL_TX_TO_RESP_RX_DLY_UUS,responseFrame,RESP_RX_TIMEOUT_UUS * UUS_TO_DWT_TIME);

    if(!isFrameGood(responseFrame, m_logger)){
        return false;
    }

    if(responseFrame.m_frame->m_functionCode == UWBMessages::FunctionCode::TWR_RESPONSE){
       // m_logger.log(LogLevel::Error,"Received response");
    }else{
        m_logger.log(LogLevel::Error,"Received NOT response");
        return false;
    }

    //  Retrieve poll transmission and response reception timestamp.
    device.getTxTimestamp(&pollTimestamp);
    responseTimestamp = responseFrame.m_rxTimestamp;

    //  Compute final message transmission time
//    finalTxTime = (responseTimestamp + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
//    finalTxTs = (((uint64_t)(finalTxTime & 0xFFFFFFFEUL)) << 8) + device.getTxAntennaDLY();

    // TEMP
    device.getSysTime(&finalTxTs);
    finalTxTs += (RESP_RX_TO_FINAL_TX_DLY_UUS/2 * UUS_TO_DWT_TIME);

    //  Construct final message
//    finalMsg.m_respMinPoll = responseTimestamp - pollTimestamp;
//    finalMsg.m_finaleMinResp = finalTxTs - responseTimestamp;

    device.finalMsgAddTs((uint8_t*)(&finalMsg.m_respMinPoll),responseTimestamp - pollTimestamp);
    device.finalMsgAddTs((uint8_t*)(&finalMsg.m_finaleMinResp),finalTxTs - responseTimestamp);

    constructUWBHeader(destinationId,UWBMessages::DATA,UWBMessages::TWR_FINAL,(uint8_t*)(&finalMsg),sizeof(finalMsg));
    return device.transmitDelayed((uint8_t*)(&finalMsg),sizeof(UWBMessages::TWRFinal),finalTxTs >> 8);
}
