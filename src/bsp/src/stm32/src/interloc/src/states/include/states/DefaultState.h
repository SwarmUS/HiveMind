#ifndef HIVE_MIND_EXAMPLESTATE_H
#define HIVE_MIND_EXAMPLESTATE_H

#include "AbstractInterlocState.h"

class DefaultState : public AbstractInterlocState {
  public:
    DefaultState(ILogger& logger, InterlocManager& interlocManager, DecawaveArray& decawaves);

    void process(InterlocStateHandler& context) override;
};

#endif // HIVE_MIND_EXAMPLESTATE_H
