#include "ApplicationInterfaceContainer.h"
#include <Mutex.h>
#include <bsp/BSPContainer.h>

ApplicationInterface& getApplicationInterface() {
    static Mutex s_mutex(10);
    static ApplicationInterface s_appInterface(BSPContainer::getUserInterface(), s_mutex);
    return s_appInterface;
}

ButtonCallbackRegister& ApplicationInterfaceContainer::getButton0CallbackRegister() {
    static ButtonCallbackRegister s_button0CallbackRegister(getApplicationInterface(),
                                                            Button::BUTTON_0);
    return s_button0CallbackRegister;
}

ButtonCallbackRegister& ApplicationInterfaceContainer::getButton1CallbackRegister() {
    static ButtonCallbackRegister s_button1CallbackRegister(getApplicationInterface(),
                                                            Button::BUTTON_1);
    return s_button1CallbackRegister;
}

ConnectionStateUI& ApplicationInterfaceContainer::getConnectionStateUI() {
    static ConnectionStateUI s_connectionStateUI(getApplicationInterface());
    return s_connectionStateUI;
}

DeviceStateUI& ApplicationInterfaceContainer::getDeviceStateUI() {
    static DeviceStateUI s_deviceStateUI(getApplicationInterface());
    return s_deviceStateUI;
}

HostHandshakeUI& ApplicationInterfaceContainer::getHostHandshakeUI() {
    static HostHandshakeUI s_hostHandshakeUI(getApplicationInterface());
    return s_hostHandshakeUI;
}

RemoteHandshakeUI& ApplicationInterfaceContainer::getRemoteHandshakeUI() {
    static RemoteHandshakeUI s_remoteHandshakeUI(getApplicationInterface());
    return s_remoteHandshakeUI;
}

UserUI& ApplicationInterfaceContainer::getUserUI() {
    static UserUI s_userUI(getApplicationInterface());
    return s_userUI;
}
