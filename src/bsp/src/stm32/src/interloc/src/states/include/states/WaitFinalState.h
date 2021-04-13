#ifndef HIVE_MIND_WAITFINALSTATE_H
#define HIVE_MIND_WAITFINALSTATE_H

#include "AbstractInterlocState.h"
#include <interloc/UWBMessages.h>
#include <interloc/UWBRxFrame.h>

class WaitFinalState : public AbstractInterlocState {
  public:
    WaitFinalState(ILogger& logger, DecawaveArray& decawaves);
    void process(InterlocStateHandler& context) override;

  private :
    UWBRxFrame m_rxFrame;

};

#endif // HIVE_MIND_WAITFINALSTATE_H
