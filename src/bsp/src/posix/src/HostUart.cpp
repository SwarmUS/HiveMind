#include "HostUart.h"

bool HostUart::send(const uint8_t* buffer, uint16_t length) {
    (void)buffer;
    (void)length;

    return true;
}

bool HostUart::isBusy() const { return false; }
