#ifndef __ANGLESENDERSTATE_H__
#define __ANGLESENDERSTATE_H__

#include "AbstractInterlocState.h"

class AngleSenderState : public AbstractInterlocState {
  public:
    AngleSenderState(ILogger& logger, DecawaveArray& decawaves);

    void process(InterlocStateHandler& context) override;

  private:
    UWBMessages::AngleMsg m_msg;
    void sendAngleFramesContinuousMode(const InterlocStateHandler& context);
    void sendAngleFramesNormalMode(const InterlocStateHandler& context);
};

#endif //__ANGLESENDERSTATE_H__
