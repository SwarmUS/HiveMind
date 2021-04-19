#ifndef HIVE_MIND_SYNCSTATE_H
#define HIVE_MIND_SYNCSTATE_H

#include "AbstractInterlocState.h"

class SyncState : public AbstractInterlocState {
  public:
    SyncState(ILogger& logger, DecawaveArray& decawaves);
    void process(InterlocStateHandler& context) override;
};

#endif // HIVE_MIND_SYNCSTATE_H
