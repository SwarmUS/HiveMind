#ifndef __BSPINTERFACEMOCK_H_
#define __BSPINTERFACEMOCK_H_

#include <bsp/IBSP.h>
#include <gmock/gmock.h>

class InterlocManagerInterfaceMock final : public IInterlocManager {
  public:
    void startInterloc() override{};

    void registerDataCallback(std::function<void(RobotPosition)> callback) override {
        m_callback = callback;
    }

    std::function<void(RobotPosition)> m_callback;
};

#endif // __BSPINTERFACEMOCK_H_
