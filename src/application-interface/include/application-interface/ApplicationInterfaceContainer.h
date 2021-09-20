#ifndef APPLICATIONINTERFACECONTAINER_H_
#define APPLICATIONINTERFACECONTAINER_H_

#include <application-interface/ApplicationInterface.h>
#include <application-interface/ButtonCallbackRegister.h>
#include <application-interface/ConnectionStateUI.h>
#include <application-interface/DeviceStateUI.h>
#include <application-interface/HostHandshakeUI.h>
#include <application-interface/RemoteHandshakeUI.h>
#include <application-interface/UserUI.h>

namespace ApplicationInterfaceContainer {
    /**@brief get the button 0 callback register*/
    ButtonCallbackRegister& getButton0CallbackRegister();

    /**@brief get the button 1 callback register*/
    ButtonCallbackRegister& getButton1CallbackRegister();

    /**@brief get the connection state UI*/
    ConnectionStateUI& getConnectionStateUI();

    /**@brief get the device state UI*/
    DeviceStateUI& getDeviceStateUI();

    /**@brief get the host handshake UI*/
    HostHandshakeUI& getHostHandshakeUI();

    /**@brief get the remote handshake UI*/
    RemoteHandshakeUI& getRemoteHandshakeUI();

    /**@brief get the UI available to the user*/
    UserUI& getUserUI();

} // namespace ApplicationInterfaceContainer

#endif // APPLICATIONINTERFACECONTAINER_H_
