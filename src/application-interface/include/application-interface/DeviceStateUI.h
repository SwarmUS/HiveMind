#ifndef DEVICESTATEUI_H_
#define DEVICESTATEUI_H_

#include <application-interface/IDeviceStateUI.h>

class DeviceStateUI : public IDeviceStateUI {
  public:
    DeviceStateUI(IApplicationInterface& appInterface);
    ~DeviceStateUI() = default;

    void setDeviceState(DeviceState state) override;

  private:
    IApplicationInterface& m_appInterface;
};

#endif // DEVICESTATEUI_H_
