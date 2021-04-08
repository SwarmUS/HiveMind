#ifndef HIVE_MIND_SENDPOLLSTATE_H
#define HIVE_MIND_SENDPOLLSTATE_H

#include "IInterlocState.h"
#include <interloc/UWBMessages.h>
#include <interloc/UWBRxFrame.h>

class SendPollState : public IInterlocState {
  public:
    void process(InterlocStateHandler& context) override;

  private:
    UWBMessages::TWRPoll m_pollMsg{};
    UWBRxFrame m_responseFrame;
};

#endif // HIVE_MIND_SENDPOLLSTATE_H
