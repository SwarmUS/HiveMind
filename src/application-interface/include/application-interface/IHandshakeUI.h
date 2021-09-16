#ifndef IHANDSHAKEUI_H_
#define IHANDSHAKEUI_H_

/**@brief Manages the interface if a device was handshaked (greeted) with or not. Lights the
 * appropriate LED to communicate the info to the user*/
class IHandshakeUI {
  public:
    virtual ~IHandshakeUI() = default;

    /**@brief Sets the handshake status of the device
     *@param handshaked if the device was handshaked or not */
    virtual void handshake(bool handshaked) = 0;
};

#endif // IHANDSHAKEUI_H_
