#ifndef IUSERUI_H_
#define IUSERUI_H_

#include <application-interface/IApplicationInterface.h>

/**@brief Manages the UI available to the user*/
class IUserUI {
  public:
    virtual ~IUserUI() = default;

    /**@brief Set the led on or off
     *@param state the state to set the led*/
    virtual void setLed(bool state) = 0;

    /**@brief Set the user seven segment ot a value
     *@param segment the value to set to the segment */
    virtual void setSegment(UserSegment segment) = 0;
};

#endif // IUSERUI_H_
