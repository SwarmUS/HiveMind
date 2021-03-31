#ifndef __INTERLOCMANAGERMOCK_H_
#define __INTERLOCMANAGERMOCK_H_

#include <bsp/IInterlocManager.h>
#include <gmock/gmock.h>

class InterlocManagerInterfaceMock final : public IInterlocManager {
  public:
    void startInterloc() override{};

    void setPositionUpdateCallback(positionUpdateCallbackFunction_t callback,
                                   void* context) override {
        m_positionUpdateCallback = callback;
        m_positionUpdateContext = context;
    }

    positionUpdateCallbackFunction_t m_callback;
    void* m_context;

    void startCalibSingleInitiator() override{};
    void startCalibSingleResponder() override{};
    void setCalibDistance(uint16_t distanceCalibCm) override { (void)distanceCalibCm; };
    void setCalibFinishedCallback(void (*fct)(void* context), void* context) override {
        (void)fct;
        (void)context;
    };
    void setCalibrationEndedCallback(calibrationEndedCallbackFunction_t callback,
                                     void* context) override {
        m_calibrationEndedCallback = callback;
        m_calibrationEndedContext = context;
    }

    positionUpdateCallbackFunction_t m_positionUpdateCallback;
    calibrationEndedCallbackFunction_t m_calibrationEndedCallback;
    void* m_positionUpdateContext;
    void* m_calibrationEndedContext;
};

#endif // __INTERLOCMANAGERMOCK_H_
