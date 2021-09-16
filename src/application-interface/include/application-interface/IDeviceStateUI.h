#ifndef IDEVICESTATEUI_H_
#define IDEVICESTATEUI_H_

#include <application-interface/IApplicationInterface.h>

/**@brief Manages UI of the device state and informs the user via the board display*/
class IDeviceStateUI {
  public:
    virtual ~IDeviceStateUI() = default;

    /**@brief Sets the state of the device
     *@param state the state to set to  */
    virtual void setDeviceState(DeviceState state) = 0;
};

#endif // IDEVICESTATEUI_H_
