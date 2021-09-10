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

void InterlocManager::setInterlocManagerStateChangeCallback(
    interlocManagerStateChangeCallbackFunction_t callback, void* context) {
    m_stateChangeCallback = callback;
    m_stateChangeCallbackContext = context;
}

void InterlocManager::setInterlocManagerRawAngleDataCallback(
    interlocRawAngleDataCallbackFunction_t callback, void* context) {
    m_rawAngleDataCallback = callback;
    m_rawAngleDataCallbackContext = context;
}

void InterlocManager::startInterloc() {
    bool allInit = true;
    for (auto& deca : m_decawaves) {
        if (!deca.init()) {
            allInit = false;
            m_logger.log(LogLevel::Warn, "InterlocManager: Could not start Decawave");
        }
    }

    syncClocks();

    if (allInit) {
        m_stateHandler.setState(InterlocStates::IDLE, InterlocEvent::NO_EVENT);
    }

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

void InterlocManager::configureTWRCalibration(uint16_t distanceCalibCm) {
    m_distanceCalibCm = distanceCalibCm;
}

uint8_t InterlocManager::powerCorrection(double twrDistance) {
    (void)twrDistance;
    return 0;
    // refer to https://confluence.swarmus.jajservers.duckdns.org/display/LOG/DECA+-++Calibration
}

void InterlocManager::updateDistance(uint16_t robotId, float distance) {
    if (m_positionUpdateCallback != nullptr) {
        InterlocUpdate update;
        update.m_robotId = robotId;
        update.m_distance = distance;
        m_positionUpdateCallback(m_positionUpdateCallbackContext, update);
    }
}

void InterlocManager::configureAngleCalibration(uint32_t numberOfFrames) {
    m_stateHandler.setAngleCalibNumberOfFrames(numberOfFrames);
}

void InterlocManager::setInterlocManagerState(InterlocStateDTO state) {
    if (m_stateChangeCallback != nullptr) {
        m_stateChangeCallback(m_stateChangeCallbackContext, m_state, state);
    }

    m_state = state;
}

void InterlocManager::sendRawAngleData(BspInterlocRawAngleData& data) {
    if (m_rawAngleDataCallback != nullptr) {
        m_rawAngleDataCallback(m_rawAngleDataCallbackContext, data);
    }

    setInterlocManagerState(InterlocStateDTO::STANDBY);
}
