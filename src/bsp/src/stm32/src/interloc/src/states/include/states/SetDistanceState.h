#ifndef HIVE_MIND_SETDISTANCESTATE_H
#define HIVE_MIND_SETDISTANCESTATE_H

#include "AbstractInterlocState.h"

class SetDistanceState : public AbstractInterlocState {
  public:
    SetDistanceState(ILogger& logger, DecawaveArray& decawaves);

    void process(InterlocStateHandler& context) override;
};

#endif // HIVE_MIND_SETDISTANCESTATE_H
