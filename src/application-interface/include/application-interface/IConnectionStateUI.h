#ifndef ICONNECTIONSTATEUI_H_
#define ICONNECTIONSTATEUI_H_

#include <application-interface/IApplicationInterface.h>

/**@brief Manages UI of the connection state with the host and informs the user via LEDs*/
class IConnectionStateUI {
  public:
    virtual ~IConnectionStateUI() = default;

    /**@brief Sets the state of the connection
     *@param state the state to set to  */
    virtual void setConnectionState(ConnectionState state) = 0;
};

#endif // ICONNECTIONSTATEUI_H_
