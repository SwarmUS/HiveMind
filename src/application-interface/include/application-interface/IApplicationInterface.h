#ifndef IAPPLICATIONINTERFACE_H_
#define IAPPLICATIONINTERFACE_H_

#include "SystemStates.h"
#include "UserStates.h"

class IApplicationInterface {
  public:
    /**@brief set the system ESP handshake and light the necessary LED*/
    virtual bool setSystemESPHandshaked(bool handshaked) = 0;

    /**@brief set the system Host handshake and light the necessary LED*/
    virtual bool setSystemHostHandshaked(bool handshaked) = 0;

    /**@brief set the system Connection state and light the necessary LED the right color*/
    virtual bool setSystemConnectionState(ConnectionState state) = 0;

    /**@brief set the system Connection state and set the seven segment accordingly*/
    virtual bool setSystemDeviceState(DeviceState state) = 0;

    /**@brief set the user LED*/
    virtual bool setUserLed(bool state) = 0;

    /**@brief set the seven segment*/
    virtual bool setSevenSegment(SevenSegment segment) = 0;

    /**@brief set the system states*/
    virtual const SystemStates& getSystemStates() const = 0;

    /**@brief get the user states*/
    virtual const UserStates& getUserStates() const = 0;
};

#endif // IAPPLICATIONINTERFACE_H_
