#include "states/SyncState.h"

SyncState::SyncState(ILogger& logger, DecawaveArray& decawaves) :
    AbstractInterlocState(logger, decawaves) {}

void SyncState::process(InterlocStateHandler& context) {}