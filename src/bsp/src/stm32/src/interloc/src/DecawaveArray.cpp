#include "interloc/DecawaveArray.h"

void DecawaveArray::initializeAll() {
    for (Decawave& deca : m_decawaves) {
        if (deca.init()) {
            m_workingDecasLength++;
        }
    }
}

bool DecawaveArray::canDoTWR() const { return m_workingDecasLength > 0; }

bool DecawaveArray::canCalculateAngles() const { return m_workingDecasLength >= 3; }