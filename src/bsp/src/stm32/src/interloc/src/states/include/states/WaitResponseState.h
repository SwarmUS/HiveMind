#ifndef HIVE_MIND_WAITRESPONSESTATE_H
#define HIVE_MIND_WAITRESPONSESTATE_H

#include "AbstractInterlocState.h"
#include <interloc/UWBMessages.h>
#include <interloc/UWBRxFrame.h>

class WaitResponseState : public AbstractInterlocState {
  public:
    WaitResponseState(ILogger& logger, DecawaveArray& decawaves);
    void process(InterlocStateHandler& context) override;

  private :
    UWBRxFrame m_rxFrame;
    UWBMessages::TWRResponse m_respMsg;

};

#endif // HIVE_MIND_WAITRESPONSESTATE_H
