#include "InterlocManager.h"
#include "bsp/BSPContainer.h"
#include <Task.h>
#define NB_CALIB_MEASUREMENTS 10

bool InterlocManager::isFrameOk(UWBRxFrame frame) {

    if (frame.m_status == UWBRxStatus::FINISHED) {
        // logger.log(LogLevel::Warn,"InterlocManager: UWB good message");
    } else if (frame.m_status == UWBRxStatus::ERROR) {
        m_logger.log(LogLevel::Warn, "InterlocManager: UWB message in error");
        return false;
    } else {
        m_logger.log(LogLevel::Warn, "InterlocManager: UWB message Timeout");
        return false;
    }
    return true;
}

InterlocManager::InterlocManager(ILogger& logger) :
    m_logger(logger), m_decaA(DW_A), m_decaB(DW_B) {}

void InterlocManager::setPositionUpdateCallback(positionUpdateCallbackFunction_t callback,
                                                void* context) {
    m_positionUpdateCallback = callback;
    m_positionUpdateCallbackContext = context;
}

void InterlocManager::startInterloc() {
    if (!m_decaA.init()) {
        m_logger.log(LogLevel::Warn, "InterlocManager: Could not start Decawave A");
    }
    if (!m_decaB.init()) {
        m_logger.log(LogLevel::Warn, "InterlocManager: Could not start Decawave B");
    }

    syncClocks();

    while (m_decaA.getState() != DW_STATE::CALIBRATED &&
           m_decaB.getState() != DW_STATE::CALIBRATED) {

        if (m_decaA.getState() == DW_STATE::RESPOND_CALIB) {
            m_logger.log(LogLevel::Info, "Starting DW calibration as responder on DW A");
            startDeviceCalibSingleResponder(0x69, m_decaA);
        } else if (m_decaA.getState() == DW_STATE::SEND_CALIB) {
            m_logger.log(LogLevel::Info, "Starting DW calibration as initiator on DW A");
            startDeviceCalibSingleInitiator(0x69, m_decaA);
        }

        if (m_decaB.getState() == DW_STATE::RESPOND_CALIB) {
            m_logger.log(LogLevel::Info, "Starting DW calibration as responder on DW B");
            startDeviceCalibSingleResponder(0x69, m_decaB);
        } else if (m_decaB.getState() == DW_STATE::SEND_CALIB) {
            m_logger.log(LogLevel::Info, "Starting DW calibration as initiator on DW B");
            startDeviceCalibSingleInitiator(0x69, m_decaB);
        }

        Task::delay(10);
    }
}

void InterlocManager::syncClocks() {
    vPortEnterCritical();

    m_decaA.setSyncMode(DW_SYNC_MODE::OSTR);
    m_decaB.setSyncMode(DW_SYNC_MODE::OSTR);

    deca_pulseSyncSignal();

    m_decaA.setSyncMode(DW_SYNC_MODE::OFF);
    m_decaB.setSyncMode(DW_SYNC_MODE::OFF);

    vPortExitCritical();
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

void InterlocManager::setCalibDistance(uint16_t distanceCalibCm) {
    m_distanceCalibCm = distanceCalibCm;
}

void InterlocManager::startCalibSingleInitiator() {
    // TODO add destination ID
    m_decaA.setState(DW_STATE::SEND_CALIB);
}

void InterlocManager::startCalibSingleResponder(uint16_t initiatorId,
                                                calibrationEndedCallbackFunction_t callback,
                                                void* context) {
    m_calibrationInitiatorId = initiatorId;
    m_calibrationEndedCallback = callback;
    m_calibrationEndedCallbackContext = context;

    // TODO add destination ID
    if (m_decaA.getState() != DW_STATE::CALIBRATED) {
        m_decaA.setState(DW_STATE::RESPOND_CALIB);
    } else if (m_decaB.getState() != DW_STATE::CALIBRATED) {
        m_decaA.setState(DW_STATE::RESPOND_CALIB);
    } else {
        // No calibration needed, notify it is ended
        callback(context, initiatorId);
    }
}

void InterlocManager::stopCalibration() {
    m_logger.log(LogLevel::Info, "Stopping DW calibration");

    if (m_decaA.getState() == DW_STATE::SEND_CALIB) {
        m_decaA.setState(DW_STATE::CONFIGURED);
    } else if (m_decaB.getState() == DW_STATE::SEND_CALIB) {
        m_decaB.setState(DW_STATE::CONFIGURED);
    }
}

void InterlocManager::startDeviceCalibSingleInitiator(uint16_t destinationId, Decawave& device) {
    device.setChannel(UWBChannel::DEFAULT_CHANNEL);
    device.setTxAntennaDLY(DEFAULT_TX_ANT_DLY);
    device.setRxAntennaDLY(DEFAULT_RX_ANT_DLY);

    while (device.getState() == DW_STATE::SEND_CALIB) { // find exit condition
        sendTWRSequence(destinationId, device);
        Task::delay(500);
    }
}

void InterlocManager::startDeviceCalibSingleResponder(uint16_t destinationId, Decawave& device) {
    int32_t error;
    int32_t dwCountOffset;
    // distance between devices in calibration mode in centimeters

    device.setChannel(UWBChannel::DEFAULT_CHANNEL);
    device.setTxAntennaDLY(DEFAULT_TX_ANT_DLY);
    device.setRxAntennaDLY(DEFAULT_RX_ANT_DLY);

    int i = 0;
    double val = 0;
    while (device.getState() == DW_STATE::RESPOND_CALIB) { // find exit condition
        val += receiveTWRSequence(destinationId, device);
        i++;
        if (i > NB_CALIB_MEASUREMENTS - 1) {

            i = 0;
            // Compute the distance error between acquired and actual
            error = (int32_t)((val * 100 / NB_CALIB_MEASUREMENTS) - m_distanceCalibCm);
            val = 0;
            // convert distance error to tick count in DW device
            // P type control, P  = 0.9
            // TODO could add a PID type control #futureImprovement
            dwCountOffset = 0.9 * error * DW_INTERNAL_CLOCK_RFEQ / SPEED_OF_LIGHT / 100;
            // antenna delay is considered equal on Rx and Tx
            device.setTxAntennaDLY((dwCountOffset >> 1) + device.getTxAntennaDLY());
            device.setRxAntennaDLY((dwCountOffset >> 1) + device.getRxAntennaDLY());
            if (dwCountOffset >> 1 <= 1) {
                device.setState(DW_STATE::CALIBRATED);
                m_calibrationEndedCallback(m_calibrationEndedCallbackContext,
                                           m_calibrationInitiatorId);
            }
        }
        Task::delay(100);
    }
}

double InterlocManager::receiveTWRSequence(uint16_t destinationId, Decawave& device) {
    UWBRxFrame rxFrame;
    uint64_t respTxTs;
    UWBMessages::TWRResponse responseMsg{};

    // receive poll message
    device.receive(rxFrame, 0);

    if (!isFrameOk(rxFrame)) {
        return -1;
    }
    if (reinterpret_cast<UWBMessages::DWFrame*>(rxFrame.m_rxBuffer.data())->m_functionCode !=
        UWBMessages::FunctionCode::TWR_POLL) {
        return -1;
    }

    uint64_t pollRxTs = rxFrame.m_rxTimestamp;

    // construct and send Response message
    constructUWBHeader(destinationId, UWBMessages::DATA, UWBMessages::TWR_RESPONSE,
                       (uint8_t*)&responseMsg, sizeof(responseMsg));
    device.transmit((uint8_t*)&responseMsg, sizeof(responseMsg));
    // TODO make the transmitDelayed work
    //    respTxTs = (pollRxTs + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME))>>8;
    //    device.transmitDelayed((uint8_t*)&responseMsg,sizeof(responseMsg),respTxTime);

    // wait for Final message reception
    device.receive(rxFrame, 0);
    if (!isFrameOk(rxFrame)) {
        return -1;
    }
    if (reinterpret_cast<UWBMessages::DWFrame*>(rxFrame.m_rxBuffer.data())->m_functionCode !=
        UWBMessages::FunctionCode::TWR_FINAL) {
        return -1;
    }

    UWBMessages::TWRFinal* finalTWRFrame =
        reinterpret_cast<UWBMessages::TWRFinal*>(rxFrame.m_rxBuffer.data());
    device.getTxTimestamp(&respTxTs);
    uint32_t finalRxTs = (uint32_t)rxFrame.m_rxTimestamp;

    // evaluate distance
    volatile uint64_t tRound1 = finalTWRFrame->m_respMinPoll;
    volatile uint32_t tRound2 = (finalRxTs - (uint32_t)respTxTs);
    volatile uint64_t tReply1 = (finalTWRFrame->m_finaleMinResp);
    volatile uint32_t tReply2 = ((uint32_t)respTxTs - (uint32_t)pollRxTs);

    volatile uint64_t num = (tRound1 * tRound2 - tReply1 * tReply2);
    volatile uint64_t denom = (tRound1 + tRound2 + tReply1 + tReply2);
    uint64_t tofDtu = num / denom;

    double tof = tofDtu * DWT_TIME_UNITS;
    double distanceEval = tof * SPEED_OF_LIGHT;

    m_logger.log(LogLevel::Info, "Measured distance : %f", distanceEval);

    return distanceEval;
}

bool InterlocManager::sendTWRSequence(uint16_t destinationId, Decawave& device) {

    UWBRxFrame responseFrame;
    UWBRxFrame finalFrame;

    UWBMessages::TWRPoll pollMsg{};
    UWBMessages::TWRFinal finalMsg{};
    uint64_t pollTimestamp;

    // Construct poll message
    constructUWBHeader(destinationId, UWBMessages::BEACON, UWBMessages::TWR_POLL,
                       (uint8_t*)&pollMsg, sizeof(pollMsg));
    device.transmitAndReceiveDelayed((uint8_t*)(&pollMsg), sizeof(UWBMessages::DWFrame),
                                     POLL_TX_TO_RESP_RX_DLY_UUS, responseFrame,
                                     RESP_RX_TIMEOUT_UUS);

    if (!isFrameOk(responseFrame)) {
        return false;
    }

    if (reinterpret_cast<UWBMessages::DWFrame*>(responseFrame.m_rxBuffer.data())->m_functionCode !=
        UWBMessages::FunctionCode::TWR_RESPONSE) {
        return false;
    }

    //  Retrieve poll transmission and response reception timestamp.
    device.getTxTimestamp(&pollTimestamp);
    uint64_t responseTimestamp = responseFrame.m_rxTimestamp;

    //  Compute final message transmission time
    uint64_t finalTxTime =
        (responseTimestamp + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
    uint64_t finalTxTs = (finalTxTime << 8); //+device.getTxAntennaDLY();

    //  Construct final message
    DecawaveUtils::tsToBytes((uint8_t*)(&finalMsg.m_respMinPoll),
                             responseTimestamp - pollTimestamp);
    DecawaveUtils::tsToBytes((uint8_t*)(&finalMsg.m_finaleMinResp), finalTxTs - responseTimestamp);

    constructUWBHeader(destinationId, UWBMessages::DATA, UWBMessages::TWR_FINAL,
                       (uint8_t*)(&finalMsg), sizeof(finalMsg));
    return device.transmitDelayed((uint8_t*)(&finalMsg), sizeof(UWBMessages::TWRFinal), finalTxTs);
    // TODO adjust all delays. Refer to DecaRangeRTLS_ARM_Source_Code_Guide
}

uint8_t InterlocManager::powerCorrection(double twrDistance) {
    (void)twrDistance;
    return 0;
    // refer to https://confluence.swarmus.jajservers.duckdns.org/display/LOG/DECA+-++Calibration
}