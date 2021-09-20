#ifndef REMOTEHANDSHAKEUI_H_
#define REMOTEHANDSHAKEUI_H_

#include <application-interface/IApplicationInterface.h>
#include <application-interface/IHandshakeUI.h>

class RemoteHandshakeUI : public IHandshakeUI {
  public:
    RemoteHandshakeUI(IApplicationInterface& appInterface);
    ~RemoteHandshakeUI() = default;

    void handshake(bool handshaked) override;

  private:
    IApplicationInterface& m_appInterface;
};

#endif // REMOTEHANDSHAKEUI_H_
