#ifndef CONNECTIONSTATEUI_H_
#define CONNECTIONSTATEUI_H_

#include <application-interface/IConnectionStateUI.h>

class ConnectionStateUI : public IConnectionStateUI {
  public:
    ConnectionStateUI(IApplicationInterface& appInterface);
    ~ConnectionStateUI() = default;

    void setConnectionState(ConnectionState state) override;

  private:
    IApplicationInterface& m_appInterface;
};

#endif // CONNECTIONSTATEUI_H_
