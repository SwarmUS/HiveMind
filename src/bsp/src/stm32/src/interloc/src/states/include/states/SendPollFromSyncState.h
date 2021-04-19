#ifndef HIVE_MIND_SENDPOLLFROMSYNCSTATE_H
#define HIVE_MIND_SENDPOLLFROMSYNCSTATE_H

#include "AbstractInterlocState.h"

class SendPollFromSyncState : public AbstractInterlocState {
    SendPollFromSyncState(ILogger& logger, DecawaveArray& decawaves);
    void process(InterlocStateHandler& context) override;
}

#endif // HIVE_MIND_SENDPOLLFROMSYNCSTATE_H
