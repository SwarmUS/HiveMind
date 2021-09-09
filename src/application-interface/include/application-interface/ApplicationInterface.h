#ifndef APPLICATIONINTERFACE_H_
#define APPLICATIONINTERFACE_H_

#include "IApplicationInterface.h"
#include <IMutex.h>
#include <bsp/IUserInterface.h>

class ApplicationInterface : public IApplicationInterface {
  public:
    ApplicationInterface(IUserInterface& userInterface, IMutex& mutex);

    ~ApplicationInterface() override = default;

    void setSystemESPHandshaked(bool handshaked) override;

    void setSystemHostHandshaked(bool handshaked) override;

    void setSystemConnectionState(ConnectionState state) override;

    void setSystemDeviceState(DeviceState state) override;

    void setUserLed(bool state) override;

    void setUserSegment(UserSegment segment) override;

    SystemStates getSystemStates() const override;

    UserStates getUserStates() const override;

    ApplicationStates getApplicationState() const override;

    static constexpr LED s_espLed = LED::LED_0;
    static constexpr LED s_hostLed = LED::LED_1;
    static constexpr LED s_userLed = LED::LED_2;

  private:
    IUserInterface& m_userInterface;
    IMutex& m_mutex;
    ApplicationStates m_states;
};

#endif // APPLICATIONINTERFACE_H_
