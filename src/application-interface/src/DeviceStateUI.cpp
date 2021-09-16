#include "DeviceStateUI.h"

DeviceStateUI::DeviceStateUI(IApplicationInterface& appInterface) : m_appInterface(appInterface) {}

void DeviceStateUI::setDeviceState(DeviceState state) {
    m_appInterface.setSystemDeviceState(state);
}
