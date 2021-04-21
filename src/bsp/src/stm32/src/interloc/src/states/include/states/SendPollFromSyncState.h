#ifndef HIVE_MIND_SENDPOLLFROMSYNCSTATE_H
#define HIVE_MIND_SENDPOLLFROMSYNCSTATE_H

#include "AbstractInterlocState.h"

class SendPollFromSyncState : public AbstractInterlocState {
  public:
    SendPollFromSyncState(ILogger& logger, DecawaveArray& decawaves);
    void process(InterlocStateHandler& context) override;

  private:
    UWBMessages::TWRPoll m_pollMsg{};
};

#endif // HIVE_MIND_SENDPOLLFROMSYNCSTATE_H
