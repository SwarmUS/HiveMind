#ifndef HIVE_MIND_SENDPOLLSTATE_H
#define HIVE_MIND_SENDPOLLSTATE_H

#include "AbstractInterlocState.h"
#include <interloc/UWBMessages.h>
#include <interloc/UWBRxFrame.h>

class SendPollState : public AbstractInterlocState {
  public:
    SendPollState(ILogger& logger, DecawaveArray& decawaves);

    void process(InterlocStateHandler& context) override;

  private:
    UWBMessages::TWRPoll m_pollMsg{};
};

#endif // HIVE_MIND_SENDPOLLSTATE_H
