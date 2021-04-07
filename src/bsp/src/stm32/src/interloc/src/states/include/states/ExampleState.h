#ifndef HIVE_MIND_EXAMPLESTATE_H
#define HIVE_MIND_EXAMPLESTATE_H

#include "IInterlocState.h"

class ExampleState : public IInterlocState {
    void process(InterlocStateHandler& context) override;
};

#endif // HIVE_MIND_EXAMPLESTATE_H
