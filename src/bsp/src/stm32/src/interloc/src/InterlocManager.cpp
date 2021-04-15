#include "interloc/InterlocManager.h"
#include "bsp/BSPContainer.h"
#include <Task.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>

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

InterlocManager::InterlocManager(ILogger& logger,
                                 InterlocStateHandler& stateHandler,
                                 DecawaveArray& decawaves) :
    m_logger(logger), m_stateHandler(stateHandler), m_decawaves(decawaves) {}

void InterlocManager::setPositionUpdateCallback(positionUpdateCallbackFunction_t callback,
                                                void* context) {
    m_positionUpdateCallback = callback;
    m_positionUpdateCallbackContext = context;
}

void InterlocManager::startInterloc() {
    for (auto& deca : m_decawaves) {
        if (!deca.init()) {
            m_logger.log(LogLevel::Warn, "InterlocManager: Could not start Decawave");
        }
    }

    syncClocks();

    // Uncomment one of the following lines to go into TWR
//     m_stateHandler.setState(InterlocStates::WAIT_POLL, InterlocEvent::NO_EVENT);
     m_stateHandler.setState(InterlocStates::SEND_POLL, InterlocEvent::NO_EVENT);

    while (true) {
        m_stateHandler.process();
    }
}

void InterlocManager::syncClocks() {
    vPortEnterCritical();

    for (auto& deca : m_decawaves) {
        deca.setSyncMode(DW_SYNC_MODE::OSTR);
    }

    deca_pulseSyncSignal();

    for (auto& deca : m_decawaves) {
        deca.setSyncMode(DW_SYNC_MODE::OFF);
    }

    vPortExitCritical();
}

void InterlocManager::setCalibDistance(uint16_t distanceCalibCm) {
    m_distanceCalibCm = distanceCalibCm;
}

void InterlocManager::startCalibSingleInitiator() {
    // TODO add destination ID
    m_logger.log(LogLevel::Warn, "Calibration disabled for now");
    // m_decaA.setState(DW_STATE::SEND_CALIB);
}

void InterlocManager::startCalibSingleResponder(uint16_t initiatorId,
                                                calibrationEndedCallbackFunction_t callback,
                                                void* context) {
    m_logger.log(LogLevel::Warn, "Calibration disabled for now");
    (void)initiatorId;
    (void)callback;
    (void)context;
    //    m_calibrationInitiatorId = initiatorId;
    //    m_calibrationEndedCallback = callback;
    //    m_calibrationEndedCallbackContext = context;
    //
    //    // TODO add destination ID
    //    if (m_decaA.getState() != DW_STATE::CALIBRATED) {
    //        m_decaA.setState(DW_STATE::RESPOND_CALIB);
    //    } else if (m_decaB.getState() != DW_STATE::CALIBRATED) {
    //        m_decaA.setState(DW_STATE::RESPOND_CALIB);
    //    } else {
    //        // No calibration needed, notify it is ended
    //        callback(context, initiatorId);
    //    }
}

void InterlocManager::stopCalibration() {
    m_logger.log(LogLevel::Warn, "Calibration disabled for now");
    //    m_logger.log(LogLevel::Info, "Stopping DW calibration");
    //
    //    if (m_decaA.getState() == DW_STATE::SEND_CALIB) {
    //        m_decaA.setState(DW_STATE::CONFIGURED);
    //    } else if (m_decaB.getState() == DW_STATE::SEND_CALIB) {
    //        m_decaB.setState(DW_STATE::CONFIGURED);
    //    }
}

// void InterlocManager::startDeviceCalibSingleInitiator(uint16_t destinationId, Decawave& device) {
//    device.setTxAntennaDLY(DEFAULT_TX_ANT_DLY);
//    device.setRxAntennaDLY(DEFAULT_RX_ANT_DLY);
//
//    while (device.getState() == DW_STATE::SEND_CALIB) { // find exit condition
//        sendTWRSequence(destinationId, device);
//        Task::delay(500);
//    }
//}
//
// void InterlocManager::startDeviceCalibSingleResponder(uint16_t destinationId, Decawave& device) {
//    int32_t error;
//    int32_t dwCountOffset;
//    // distance between devices in calibration mode in centimeters
//    device.setTxAntennaDLY(DEFAULT_TX_ANT_DLY);
//    device.setRxAntennaDLY(DEFAULT_RX_ANT_DLY);
//
//    int i = 0;
//    double val = 0;
//    while (device.getState() == DW_STATE::RESPOND_CALIB) { // find exit condition
//        double calculatedDistance;
//        do {
//            calculatedDistance = receiveTWRSequence(destinationId, device);
//            Task::delay(200);
//        } while (calculatedDistance < 0);
//
//        val += receiveTWRSequence(destinationId, device);
//        i++;
//        if (i > NB_CALIB_MEASUREMENTS - 1) {
//
//            i = 0;
//            // Compute the distance error between acquired and actual
//            error = (int32_t)((val * 100 / NB_CALIB_MEASUREMENTS) - m_distanceCalibCm);
//            val = 0;
//            // convert distance error to tick count in DW device
//            // P type control, P  = 0.9
//            // TODO could add a PID type control #futureImprovement
//            dwCountOffset = 0.9 * error * DW_INTERNAL_CLOCK_RFEQ / SPEED_OF_LIGHT / 100;
//            // antenna delay is considered equal on Rx and Tx
//            device.setTxAntennaDLY((dwCountOffset >> 1) + device.getTxAntennaDLY());
//            device.setRxAntennaDLY((dwCountOffset >> 1) + device.getRxAntennaDLY());
//            if (dwCountOffset >> 1 <= 1) {
//                device.setState(DW_STATE::CALIBRATED);
//                m_calibrationEndedCallback(m_calibrationEndedCallbackContext,
//                                           m_calibrationInitiatorId);
//            }
//        }
//        Task::delay(100);
//    }
//}

uint8_t InterlocManager::powerCorrection(double twrDistance) {
    (void)twrDistance;
    return 0;
    // refer to https://confluence.swarmus.jajservers.duckdns.org/display/LOG/DECA+-++Calibration
}