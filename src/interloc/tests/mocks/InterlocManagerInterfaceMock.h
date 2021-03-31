#ifndef __INTERLOCMANAGERMOCK_H_
#define __INTERLOCMANAGERMOCK_H_

#include <bsp/IInterlocManager.h>
#include <gmock/gmock.h>

class InterlocManagerInterfaceMock final : public IInterlocManager {
  public:
    void startInterloc() override{};

    void setPositionUpdateCallback(positionUpdateCallbackFunction_t callback,
                                   void* context) override {
        m_callback = callback;
        m_context = context;
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
};

#endif // __INTERLOCMANAGERMOCK_H_
