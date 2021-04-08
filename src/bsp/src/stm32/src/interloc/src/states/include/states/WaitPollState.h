#ifndef HIVE_MIND_WAITPOLLSTATE_H
#define HIVE_MIND_WAITPOLLSTATE_H

#include "IInterlocState.h"
#include <interloc/UWBMessages.h>
#include <interloc/UWBRxFrame.h>

class WaitPollState : public IInterlocState {
  public:
    void process(InterlocStateHandler& context) override;

  private:
    UWBRxFrame m_rxFrame;
};

#endif // HIVE_MIND_WAITPOLLSTATE_H
