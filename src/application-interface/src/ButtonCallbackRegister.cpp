#include "ButtonCallbackRegister.h"

ButtonCallbackRegister::ButtonCallbackRegister(IApplicationInterface& appInterface, Button button) :
    m_appInterface(appInterface), m_button(button) {}

void ButtonCallbackRegister::setCallback(buttonCallbackFunction_t callback, void* context) {
    m_appInterface.setSystemButtonCallback(m_button, callback, context);
}
