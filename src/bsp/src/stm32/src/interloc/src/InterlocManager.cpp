#include "interloc/InterlocManager.h"
#include "bsp/BSPContainer.h"
#include <BSP.h>
#include <Task.h>
#include <interloc/InterlocBSPContainer.h>
#include <states/InterlocStateContainer.h>

#define NB_CALIB_MEASUREMENTS 10

InterlocManager::InterlocManager(ILogger& logger,
                                 InterlocStateHandler& stateHandler,
                                 DecawaveArray& decawaves,
                                 INotificationQueue<InterlocUpdate>& interlocUpdateQueue,
                                 IButtonCallbackRegister& buttonCallbackRegister) :
    m_logger(logger),
    m_stateHandler(stateHandler),
    m_buttonCallbackRegister(buttonCallbackRegister),
    m_decawaves(decawaves),
    m_interlocUpdateQueue(interlocUpdateQueue) {
    m_buttonCallbackRegister.setCallback(staticButtonCallback, this);
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

void InterlocManager::updateAngleCalculatorParameters(
    const ConfigureAngleParametersDTO& newParams) {
    AngleCalculatorParameters& oldParams =
        ((BSP&)BSPContainer::getBSP()).getStorage().getAngleCaculatorParameters();

    if (newParams.getAntennasLength() != 2) {
        m_logger.log(LogLevel::Error, "Angle Params Update: Pair does not have 2 antennas");
        return;
    }

    if (newParams.getPairId() >= NUM_ANTENNA_PAIRS) {
        m_logger.log(LogLevel::Error, "Angle Params Update: Pair ID >= than # of pairs");
        return;
    }

    if (newParams.getSlopeDecisionLength() != NUM_ANTENNA_PAIRS) {
        m_logger.log(LogLevel::Error,
                     "Angle Params Update: Slope decision not length of # of pairs");
        return;
    }

    if (newParams.getTdoaSlopesLength() > NUM_TDOA_SLOPES ||
        newParams.getTdoaInterceptsLength() > NUM_TDOA_SLOPES) {
        m_logger.log(LogLevel::Error, "Angle Params Update: TDOA params not correct length");
        return;
    }

    if (newParams.getPdoaInterceptsLength() > NUM_PDOA_SLOPES ||
        newParams.getPdoaOriginsLength() > NUM_PDOA_SLOPES) {
        m_logger.log(LogLevel::Error, "Angle Params Update: PDOA params not correct length");
        return;
    }

    oldParams.m_antennaPairs[newParams.getPairId()][0] = newParams.getAntennas()[0];
    oldParams.m_antennaPairs[newParams.getPairId()][1] = newParams.getAntennas()[1];
    for (unsigned int i = 0; i < newParams.getSlopeDecisionLength(); i++) {
        oldParams.m_slopeDecisionMatrix[newParams.getPairId()][i] = newParams.getSlopeDecision()[i];
    }

    oldParams.m_tdoaNormalizationFactors[newParams.getPairId()] =
        newParams.getTdoaNormalizationFactor();

    for (unsigned int i = 0; i < newParams.getTdoaInterceptsLength(); i++) {
        oldParams.m_tdoaSlopes[newParams.getPairId()][i] = newParams.getTdoaSlopes()[i];
        oldParams.m_tdoaIntercepts[newParams.getPairId()][i] = newParams.getTdoaIntercepts()[i];
    }

    oldParams.m_pdoaNormalizationFactors[newParams.getPairId()] =
        newParams.getPdoaNormalizationFactor();
    oldParams.m_pdoaSlopes[newParams.getPairId()] = newParams.getPdoaSlope();

    for (unsigned int i = 0; i < newParams.getPdoaOriginsLength(); i++) {
        oldParams.m_pdoaIntercepts[newParams.getPairId()][i] = newParams.getPdoaIntercepts()[i];
        oldParams.m_pdoaOrigins[newParams.getPairId()][i] = newParams.getPdoaOrigins()[i];
    }

    oldParams.m_parametersValidSecretNumbers[newParams.getPairId()] =
        ANGLE_PARAMETERS_VALID_SECRET_NUMBER;

    taskENTER_CRITICAL();
    bool ret = ((BSP&)BSPContainer::getBSP()).getStorage().saveToFlash();
    taskEXIT_CRITICAL();
    InterlocBSPContainer::getAngleCalculator().setCalculatorParameters(oldParams);

    if (!ret) {
        m_logger.log(LogLevel::Error, "Angle Params Update: Error while saving to flash");
    } else {
        m_logger.log(LogLevel::Info, "Updated angle params for pair %d", newParams.getPairId());
    }
}

void InterlocManager::startInterloc() {
    m_decawaves.initializeAll();

    syncClocks();

    if (m_decawaves.canDoTWR()) {
        m_state = InterlocStateDTO::OPERATING;
    } else {
        m_logger.log(LogLevel::Error, "Could not initialize enough Decawaves to do TWR");
        m_state = InterlocStateDTO::STANDBY;
    }

    m_stateHandler.setState(InterlocStates::IDLE, InterlocEvent::NO_EVENT);

    while (true) {
        m_stateHandler.process();
    }
}

void InterlocManager::syncClocks() {
    vPortEnterCritical();

    for (auto& deca : m_decawaves) {
        if (deca.isReady()) {
            deca.setSyncMode(DW_SYNC_MODE::OSTR);
        }
    }

    deca_pulseSyncSignal();

    for (auto& deca : m_decawaves) {
        if (deca.isReady()) {
            deca.setSyncMode(DW_SYNC_MODE::OFF);
        }
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

void InterlocManager::updateInterloc(uint16_t robotId,
                                     std::optional<float> distance,
                                     std::optional<float> angle) {
    if (!m_interlocUpdateQueue.isFull()) {
        InterlocUpdate update;
        update.m_robotId = robotId;
        update.m_distance = distance;
        update.m_angleOfArrival = angle;

        m_interlocUpdateQueue.push(update);
    } else {
        m_logger.log(LogLevel::Warn, "Could not push interloc update to queue because it is full");
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

InterlocStateDTO InterlocManager::getState() const { return m_state; }

void InterlocManager::staticButtonCallback(void* context) {
    static_cast<InterlocManager*>(context)->buttonCallback();
}

void InterlocManager::buttonCallback() { m_state = InterlocStateDTO::ANGLE_CALIB_SENDER; }
