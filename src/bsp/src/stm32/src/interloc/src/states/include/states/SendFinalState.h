#ifndef HIVE_MIND_SENDFINALSTATE_H
#define HIVE_MIND_SENDFINALSTATE_H

#include "IInterlocState.h"
#include <interloc/UWBMessages.h>
#include <interloc/UWBRxFrame.h>

class SendFinalState : public IInterlocState {
  public:
    void process(InterlocStateHandler& context) override;

  private:
    UWBMessages::TWRFinal m_finalMsg{};
};

#endif // HIVE_MIND_SENDFINALSTATE_H
