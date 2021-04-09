#include "states/AbstractInterlocState.h"
#include <interloc/InterlocBSPContainer.h>

AbstractInterlocState::AbstractInterlocState(ILogger& logger, DecawaveArray& decawaves) :
    m_logger(logger), m_decawaves(decawaves) {}
