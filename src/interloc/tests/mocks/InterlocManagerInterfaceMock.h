#ifndef __BSPINTERFACEMOCK_H_
#define __BSPINTERFACEMOCK_H_

#include <bsp/IBSP.h>
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
};

#endif // __BSPINTERFACEMOCK_H_
