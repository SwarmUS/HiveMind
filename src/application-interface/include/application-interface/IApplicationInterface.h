#ifndef IAPPLICATIONINTERFACE_H_
#define IAPPLICATIONINTERFACE_H_

#include <application-interface/ApplicationStates.h>
#include <bsp/IUserInterface.h>

class IApplicationInterface {
  public:
    virtual ~IApplicationInterface() = default;

    /**@brief set the system Remote handshake and light the necessary LED*/
    virtual void setSystemRemoteHandshaked(bool handshaked) = 0;

    /**@brief set the system Host handshake and light the necessary LED*/
    virtual void setSystemHostHandshaked(bool handshaked) = 0;

    /**@brief set the system Connection state and light the necessary LED the right color*/
    virtual void setSystemConnectionState(ConnectionState state) = 0;

    /**@brief set the system Connection state and set the seven segment accordingly*/
    virtual void setSystemDeviceState(DeviceState state) = 0;

    /**@brief set the systems button to a callback*/
    virtual void setSystemButtonCallback(Button button,
                                         buttonCallbackFunction_t callback,
                                         void* context) = 0;

    /**@brief set the user LED*/
    virtual void setUserLed(bool state) = 0;

    /**@brief set the user seven segment*/
    virtual void setUserSegment(UserSegment segment) = 0;

    /**@brief set the system states*/
    virtual SystemStates getSystemStates() const = 0;

    /**@brief get the user states*/
    virtual UserStates getUserStates() const = 0;

    /**@brief get the application states*/
    virtual ApplicationStates getApplicationState() const = 0;
};

#endif // IAPPLICATIONINTERFACE_H_
