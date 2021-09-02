#ifndef APPLICATIONINTERFACE_H_
#define APPLICATIONINTERFACE_H_

#include "IApplicationInterface.h"

class ApplicationInterface : public IApplicationInterface {
  public:
    bool setSystemESPHandshaked(bool handshaked) override;

    bool setSystemHostHandshaked(bool handshaked) override;

    bool setSystemConnectionState(ConnectionState state) override;

    bool setSystemDeviceState(DeviceState state) override;

    bool setUserLed(bool state) override;

    bool setSevenSegment(SevenSegment segment) override;

    const SystemStates& getSystemStates() const override;

    const UserStates& getUserStates() const override;

  private:
    UserStates m_userStates;
    SystemStates m_systemStates;
};

#endif // APPLICATIONINTERFACE_H_
