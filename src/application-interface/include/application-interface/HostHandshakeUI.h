#ifndef HOSTHANDSHAKEUI_H_
#define HOSTHANDSHAKEUI_H_

#include <application-interface/IApplicationInterface.h>
#include <application-interface/IHandshakeUI.h>

class HostHandshakeUI : public IHandshakeUI {
  public:
    HostHandshakeUI(IApplicationInterface& appInterface);
    ~HostHandshakeUI() override = default;

    void handshake(bool handshaked) override;
    private:
    IApplicationInterface& m_appInterface;
};

#endif // HOSTHANDSHAKEUI_H_
