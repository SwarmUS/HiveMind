#ifndef HIVE_MIND_WAITPOLLSTATE_H
#define HIVE_MIND_WAITPOLLSTATE_H

#include "AbstractInterlocState.h"
#include <interloc/UWBMessages.h>
#include <interloc/UWBRxFrame.h>

class WaitPollState : public AbstractInterlocState {
  public:
    WaitPollState(ILogger& logger, DecawaveArray& decawaves);

    void process(InterlocStateHandler& context) override;

  private:
    UWBRxFrame m_rxFrame;
};

#endif // HIVE_MIND_WAITPOLLSTATE_H
