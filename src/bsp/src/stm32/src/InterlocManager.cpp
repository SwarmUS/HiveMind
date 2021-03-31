#include "InterlocManager.h"
#include "bsp/BSPContainer.h"
#include <Task.h>
#define NB_CALIB_TRIES 10

bool isFrameGood(UWBRxFrame frame,ILogger&  logger){

    if(frame.m_status == UWBRxStatus::FINISHED) {
        //logger.log(LogLevel::Warn,"Good message");
    }else if(frame.m_status == UWBRxStatus::ERROR){
        logger.log(LogLevel::Warn,"Message in error");
        return false;
    }else{
        logger.log(LogLevel::Warn,"Message Timeout");
        return false;
    }
    return true;
}

void InterlocManager::setCalibFinishedCallback(void (*fct)(void* context), void* context){
    m_calibFinishedCallback = fct;
    m_calibFinishedCallbackContext = context;
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

    while(m_decaA.getState() != DW_STATE::CALIBRATED && m_decaB.getState() != DW_STATE::CALIBRATED) {
        if (m_decaA.getState() == DW_STATE::RESPOND_CALIB) {
            startCalibSingleRespond(0x69, m_decaA);
        } else if (m_decaA.getState() == DW_STATE::SEND_CALIB) {
            startCalibSingleInit(0x69, m_decaA);
        }

        if (m_decaB.getState() == DW_STATE::RESPOND_CALIB) {
            startCalibSingleRespond(0x69, m_decaB);
        } else if (m_decaB.getState() == DW_STATE::SEND_CALIB) {
            startCalibSingleInit(0x69, m_decaB);
        }
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

void InterlocManager::setCalibDistance(uint16_t distanceForCalibCm){
    m_distanceForCalibCm = distanceForCalibCm;
}

void InterlocManager::startCalibSingleInit() {
    // TODO add destination ID
        m_decaA.setState(DW_STATE::SEND_CALIB);
}

void InterlocManager::startCalibSingleRespond() {
    // TODO add destination ID
    if (m_decaA.getState() != DW_STATE::CALIBRATED) {
        m_decaA.setState(DW_STATE::RESPOND_CALIB);
    }else if (m_decaB.getState() != DW_STATE::CALIBRATED) {
        m_decaA.setState(DW_STATE::RESPOND_CALIB);
    }
}

void InterlocManager::startCalibSingleInit(uint16_t destinationId, Decawave& device) {
    device.setChannel(UWBChannel::CHANNEL_2);
    device.setTxAntennaDLY(DEFAULT_TX_ANT_DLY);
    device.setRxAntennaDLY(DEFAULT_RX_ANT_DLY);

    while(device.getState() == DW_STATE::SEND_CALIB) { //find exit condition
        sendTWRSequence(destinationId, device);
        Task::delay(100);
    }

}

void InterlocManager::startCalibSingleRespond(uint16_t destinationId, Decawave& device){
    int32_t error;
    int32_t dwCountOffset;
//distance between devices in calibration mode in centimeters

    device.setChannel(UWBChannel::CHANNEL_2);
    device.setTxAntennaDLY(DEFAULT_TX_ANT_DLY);
    device.setRxAntennaDLY(DEFAULT_RX_ANT_DLY);

    int i = 0;
    double val = 0;
    while(device.getState() == DW_STATE::RESPOND_CALIB) { //find exit condition
        val += receiveTWRSequence(destinationId, device);
        i++;
        if(i > NB_CALIB_TRIES-1){

            i = 0;
            // Compute the distance error between acquired and actual
            error = (int32_t)((val*100/NB_CALIB_TRIES) - m_distanceForCalibCm);
            val = 0;
            // convert distance error to tick count in DW device
            // P type control, P  = 0.9
            // TODO could had a PID type control #futureImprovement
            dwCountOffset = 0.9 * error * DW_INTERNAL_CLOCK_RFEQ / SPEED_OF_LIGHT /100;
            // antenna delay is considered equal on Rx and Tx
            device.setTxAntennaDLY((dwCountOffset>>1)+device.getTxAntennaDLY());
            device.setRxAntennaDLY((dwCountOffset>>1)+device.getRxAntennaDLY());
            if(dwCountOffset>>1 <=1){
                device.setState(DW_STATE::CALIBRATED);
                m_calibFinishedCallback(m_calibFinishedCallbackContext);
            }
        }
        Task::delay(50);
    }
}

double InterlocManager::receiveTWRSequence(uint16_t destinationId, Decawave& device) {
    UWBRxFrame rxFrame;
    uint64_t pollRxTs;
    uint64_t respTxTs;
    uint64_t finalRxTs;
    UWBMessages::TWRResponse responseMsg{};
    double varA;
    double varB;
    double varC;
    double varD;
    double distanceEval;
    double tof;
    uint64_t tofDtu;

    // receive poll message
    device.receive(rxFrame, 0);

    if(!isFrameGood(rxFrame, m_logger)){
        return 0;
    }
    if(rxFrame.m_frame->m_functionCode != UWBMessages::FunctionCode::TWR_POLL){
        return 0;
    }

    pollRxTs = rxFrame.m_rxTimestamp;

    // construct and send Response message
    constructUWBHeader(destinationId, UWBMessages::DATA, UWBMessages::TWR_RESPONSE,(uint8_t*)&responseMsg, sizeof(responseMsg));
    //    respTxTs = (pollRxTs + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME))>>8;

    device.transmit((uint8_t*)&responseMsg,sizeof(responseMsg));
    //    device.transmitDelayed((uint8_t*)&responseMsg,sizeof(responseMsg),respTxTime);

    // wait for Final message reception
    device.receive(rxFrame, 0);
    if(!isFrameGood(rxFrame, m_logger)){
        return 0;
    }
    if(rxFrame.m_frame->m_functionCode != UWBMessages::FunctionCode::TWR_FINAL){
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

    return distanceEval;
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
    device.transmitAndReceiveDelayed((uint8_t*)(&pollMsg), sizeof(UWBMessages::DWFrame),POLL_TX_TO_RESP_RX_DLY_UUS,responseFrame,RESP_RX_TIMEOUT_UUS);

    if(!isFrameGood(responseFrame, m_logger)){
        return false;
    }

    if(responseFrame.m_frame->m_functionCode != UWBMessages::FunctionCode::TWR_RESPONSE){
        return false;
    }

    //  Retrieve poll transmission and response reception timestamp.
    device.getTxTimestamp(&pollTimestamp);
    responseTimestamp = responseFrame.m_rxTimestamp;

    //  Compute final message transmission time
    finalTxTime = (responseTimestamp + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    finalTxTs = (finalTxTime << 8); //+device.getTxAntennaDLY();

    //  Construct final message
    device.finalMsgAddTs((uint8_t*)(&finalMsg.m_respMinPoll),responseTimestamp - pollTimestamp);
    device.finalMsgAddTs((uint8_t*)(&finalMsg.m_finaleMinResp),finalTxTs - responseTimestamp);

    constructUWBHeader(destinationId,UWBMessages::DATA,UWBMessages::TWR_FINAL,(uint8_t*)(&finalMsg),sizeof(finalMsg));
    return device.transmitDelayed((uint8_t*)(&finalMsg),sizeof(UWBMessages::TWRFinal),finalTxTs);
}
