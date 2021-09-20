#include "RemoteHandshakeUI.h"

RemoteHandshakeUI::RemoteHandshakeUI(IApplicationInterface& appInterface) :
    m_appInterface(appInterface) {}

void RemoteHandshakeUI::handshake(bool handshaked) {
    m_appInterface.setSystemRemoteHandshaked(handshaked);
}
