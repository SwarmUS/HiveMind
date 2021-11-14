#ifndef HIVE_MIND_SETDISTANCESTATE_H
#define HIVE_MIND_SETDISTANCESTATE_H

#include "AbstractInterlocState.h"
#include <cstdint>

class UpdateInterloc : public AbstractInterlocState {
  public:
    UpdateInterloc(ILogger& logger, DecawaveArray& decawaves);

    void process(InterlocStateHandler& context) override;
};

#endif // HIVE_MIND_SETDISTANCESTATE_H
