#ifndef IBUTTONCALLBACKREGISTER_H_
#define IBUTTONCALLBACKREGISTER_H_

#include <application-interface/IApplicationInterface.h>

/**@brief Manages the callback of a single button*/
class IButtonCallbackRegister {
  public:
    virtual ~IButtonCallbackRegister() = default;

    /**@brief sets the callback to the managed button
     *@param callback a pointer to the callback function
     *@param context a pointer to the context passed to the callback when called*/
    virtual void setCallback(buttonCallbackFunction_t callback, void* context) = 0;
};

#endif // IBUTTONCALLBACKREGISTER_H_
