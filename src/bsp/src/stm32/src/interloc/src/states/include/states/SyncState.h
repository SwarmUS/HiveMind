#ifndef HIVE_MIND_SYNCSTATE_H
#define HIVE_MIND_SYNCSTATE_H

#include "AbstractInterlocState.h"

class SyncState : public AbstractInterlocState {
  public:
    SyncState(ILogger& logger, DecawaveArray& decawaves);
    void process(InterlocStateHandler& context) override;

  private:
    UWBRxFrame m_rxFrame;

    void handlePollReceived(InterlocStateHandler& context);
    void capTimeoutUint16(uint32_t& remainingRxTimeoutUs, uint16_t& rxTimeoutUs) const;
};

#endif // HIVE_MIND_SYNCSTATE_H
