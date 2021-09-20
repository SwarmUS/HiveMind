#ifndef BUTTONCALLBACKREGISTER_H_
#define BUTTONCALLBACKREGISTER_H_

#include <application-interface/IButtonCallbackRegister.h>

class ButtonCallbackRegister : public IButtonCallbackRegister {
  public:
    ButtonCallbackRegister(IApplicationInterface& appInterface, Button button);
    ~ButtonCallbackRegister() = default;

    void setCallback(buttonCallbackFunction_t callback, void* context) override;

  private:
    IApplicationInterface& m_appInterface;
    Button m_button;
};

#endif // BUTTONCALLBACKREGISTER_H_
