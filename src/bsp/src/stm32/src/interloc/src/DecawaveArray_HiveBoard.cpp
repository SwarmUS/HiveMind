#include "interloc/DecawaveArray.h"

std::optional<std::reference_wrapper<Decawave>> DecawaveArray::getMasterAntenna() {
    if (!canDoTWR()) {
        return {};
    }

    // Three antennas are plugged so use center ports
    if (canCalculateAngles()) {
        if (m_decawaves[DW_B0].isReady()) {
            return m_decawaves[DW_B0];
        }
        if (m_decawaves[DW_B1].isReady()) {
            return m_decawaves[DW_B1];
        }
    }

    // If nothing is plugged in the center (or only one antenna is plugged) return the first plugged
    // antenna
    for (Decawave& deca : m_decawaves) {
        if (deca.isReady()) {
            return deca;
        }
    }

    return {};
}

std::optional<std::reference_wrapper<Decawave>> DecawaveArray::getLeftAntenna() {
    if (!canCalculateAngles()) {
        return {};
    }

    if (m_decawaves[DW_C0].isReady()) {
        return m_decawaves[DW_C0];
    }
    if (m_decawaves[DW_C1].isReady()) {
        return m_decawaves[DW_C1];
    }

    return {};
}

std::optional<std::reference_wrapper<Decawave>> DecawaveArray::getRightAntenna() {
    if (!canCalculateAngles()) {
        return {};
    }

    if (m_decawaves[DW_A0].isReady()) {
        return m_decawaves[DW_A0];
    }
    if (m_decawaves[DW_A1].isReady()) {
        return m_decawaves[DW_A1];
    }

    return {};
}

void DecawaveArray::initializeAngleAntennaArray() {
    m_angleAntennaArray[0] = getLeftAntenna();
    m_angleAntennaArray[1] = getMasterAntenna();
    m_angleAntennaArray[2] = getRightAntenna();
}