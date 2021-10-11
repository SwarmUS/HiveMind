#include "interloc/DecawaveArray.h"

std::optional<std::reference_wrapper<Decawave>> DecawaveArray::getMasterAntenna() {
    if (!canDoTWR()) {
        return {};
    }

    for (Decawave& deca : m_decawaves) {
        if (deca.isReady()) {
            return deca;
        }
    }

    return {};
}

std::optional<std::reference_wrapper<Decawave>> DecawaveArray::getLeftAntenna() { return {}; }

std::optional<std::reference_wrapper<Decawave>> DecawaveArray::getRightAntenna() { return {}; }

void DecawaveArray::initializeAngleAntennaArray() {}