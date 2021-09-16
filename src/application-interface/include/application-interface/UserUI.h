#ifndef USERUI_H_
#define USERUI_H_

#include <application-interface/IUserUI.h>

class UserUI : public IUserUI {
  public:
    UserUI(IApplicationInterface& appInterface);

    ~UserUI() = default;

    void setLed(bool state) override;

    void setSegment(UserSegment segment) override;

  private:
    IApplicationInterface& m_appInterface;
};

#endif // USERUI_H_
