#include "states/AbstractInterlocState.h"
#include <interloc/InterlocBSPContainer.h>

AbstractInterlocState::AbstractInterlocState(ILogger& logger,
                                             InterlocManager& interlocManager,
                                             DecawaveArray& decawaves) :
    m_logger(logger), m_interlocManager(interlocManager), m_decawaves(decawaves) {}
