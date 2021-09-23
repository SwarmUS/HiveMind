#ifndef __ANGLESENDERSTATE_H__
#define __ANGLESENDERSTATE_H__

#include "AbstractInterlocState.h"

class AngleSenderState : public AbstractInterlocState {
  public:
    constexpr static uint8_t m_transmissionContingencyFactor = 3;

    AngleSenderState(ILogger& logger, DecawaveArray& decawaves);

    void process(InterlocStateHandler& context) override;

  private:
    UWBMessages::AngleMsg m_msg;
    void sendAngleFrames(const InterlocStateHandler& context);
};

#endif //__ANGLESENDERSTATE_H__
