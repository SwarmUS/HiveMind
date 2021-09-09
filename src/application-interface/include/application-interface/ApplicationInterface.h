#ifndef APPLICATIONINTERFACE_H_
#define APPLICATIONINTERFACE_H_

#include "IApplicationInterface.h"
#include <bsp/IUserInterface.h>
#include <os/mutex.h>

class ApplicationInterface : public IApplicationInterface {
  public:
    ApplicationInterface(IUserInterface& userInterface);

    void setSystemESPHandshaked(bool handshaked) override;

    void setSystemHostHandshaked(bool handshaked) override;

    void setSystemConnectionState(ConnectionState state) override;

    void setSystemDeviceState(DeviceState state) override;

    void setUserLed(bool state) override;

    void setSevenSegment(SevenSegment segment) override;

    const SystemStates& getSystemStates() const override;

    const UserStates& getUserStates() const override;

  private:
    static constexpr LED s_espLed = LED::LED_0;
    static constexpr LED s_hostLed = LED::LED_0;

    IUserInterface& m_userInterface;
    UserStates m_userStates;
    SystemStates m_systemStates;
};

#endif // APPLICATIONINTERFACE_H_
