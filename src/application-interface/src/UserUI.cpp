#include "UserUI.h"

UserUI::UserUI(IApplicationInterface& appInterface) : m_appInterface(appInterface) {}

void UserUI::setLed(bool state) { m_appInterface.setUserLed(state); }

void UserUI::setSegment(UserSegment segment) { m_appInterface.setUserSegment(segment); }
