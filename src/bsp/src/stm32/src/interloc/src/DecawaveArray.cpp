#include "interloc/DecawaveArray.h"

void DecawaveArray::initializeAll() {
    for (Decawave& deca : m_decawaves) {
        if (deca.init()) {
            m_workingDecasLength++;
        }
    }

    initializeAngleAntennaArray();
}

bool DecawaveArray::canDoTWR() const { return m_workingDecasLength > 0; }

bool DecawaveArray::canCalculateAngles() const { return m_workingDecasLength >= 3; }

std::array<std::optional<std::reference_wrapper<Decawave>>, DecawaveArray::angleAntennaArraySize>& DecawaveArray::
    getAngleAntennaArray() {
    return m_angleAntennaArray;
}