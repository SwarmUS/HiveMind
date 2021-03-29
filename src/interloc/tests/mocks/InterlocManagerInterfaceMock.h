#ifndef __BSPINTERFACEMOCK_H_
#define __BSPINTERFACEMOCK_H_

#include <bsp/IBSP.h>
#include <gmock/gmock.h>

class InterlocManagerInterfaceMock final : public IInterlocManager {
  public:
    void startInterloc() override{};

    void registerPositionUpdateCallback(std::function<void(InterlocUpdate)> callback) override {
        m_callback = callback;
    }

    std::function<void(InterlocUpdate)> m_callback;
};

#endif // __BSPINTERFACEMOCK_H_
