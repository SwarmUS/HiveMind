#include "HostHandshakeUI.h"

HostHandshakeUI::HostHandshakeUI(IApplicationInterface& appInterface) :
    m_appInterface(appInterface) {}

void HostHandshakeUI::handshake(bool handshaked) {
    m_appInterface.setSystemHostHandshaked(handshaked);
}
