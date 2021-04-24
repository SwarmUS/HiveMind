#ifndef __INTERLOCMANAGERMOCK_H_
#define __INTERLOCMANAGERMOCK_H_

#include <bsp/IInterlocManager.h>
#include <gmock/gmock.h>

class InterlocManagerInterfaceMock : public IInterlocManager {
  public:
    void startInterloc() override{};

    void setPositionUpdateCallback(positionUpdateCallbackFunction_t callback,
                                   void* context) override {
        m_positionUpdateCallback = callback;
        m_positionUpdateContext = context;
    }

    MOCK_METHOD(void, startCalibSingleInitiator, (), (override));
    MOCK_METHOD(void, stopCalibration, (), (override));
    MOCK_METHOD(void,
                startCalibSingleResponder,
                (uint16_t initiatorId, calibrationEndedCallbackFunction_t callback, void* context),
                (override));
    MOCK_METHOD(void, setCalibDistance, (uint16_t distanceCalibCm), (override));

    positionUpdateCallbackFunction_t m_positionUpdateCallback;
    void* m_positionUpdateContext;
};

#endif // __INTERLOCMANAGERMOCK_H_
