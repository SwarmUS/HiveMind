#ifndef __INTERLOCMANAGERMOCK_H_
#define __INTERLOCMANAGERMOCK_H_

#include <bsp/IInterlocManager.h>
#include <gmock/gmock.h>

class InterlocManagerInterfaceMock : public IInterlocManager {
  public:
    void startInterloc() override{};

    void setInterlocManagerStateChangeCallback(
        interlocManagerStateChangeCallbackFunction_t callback, void* context) override {
        m_stateChangeCallback = callback;
        m_stateChangeContext = context;
    }

    void setInterlocManagerRawAngleDataCallback(interlocRawAngleDataCallbackFunction_t callback,
                                                void* context) override {
        m_angleDataCallback = callback;
        m_angleDataContext = context;
    }

    MOCK_METHOD(void, setInterlocManagerState, (InterlocStateDTO state), (override));
    MOCK_METHOD(void, configureTWRCalibration, (uint16_t distanceCalibCm), (override));
    MOCK_METHOD(void, configureAngleCalibration, (uint32_t numberOfFrames), (override));
    MOCK_METHOD(void,
                updateAngleCalculatorParameters,
                (const ConfigureAngleParametersDTO& newParams),
                (override));

    interlocManagerStateChangeCallbackFunction_t m_stateChangeCallback;
    void* m_stateChangeContext;

    interlocRawAngleDataCallbackFunction_t m_angleDataCallback;
    void* m_angleDataContext;
};

#endif // __INTERLOCMANAGERMOCK_H_
