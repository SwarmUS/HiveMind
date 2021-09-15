#include "ConnectionStateUI.h"

ConnectionStateUI::ConnectionStateUI(IApplicationInterface& appInterface) :
    m_appInterface(appInterface) {}

void ConnectionStateUI::setConnectionState(ConnectionState state) {
    m_appInterface.setSystemConnectionState(state);
}
