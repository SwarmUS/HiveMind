#include "PhoneCommunication.h"
#include "hal/uart_phone.h"
#include <cstdio>
#include <cstring>

void PhoneCommunication::sendBytes(const uint8_t* bytes, uint16_t length) {
    *txBuffer = length;
    memcpy((txBuffer + sizeof(length)), bytes, length);

    bool ret = UartPhone_transmitBuffer(txBuffer, length + sizeof(length));
    if (!ret) {
        printf("Could not transmit to phone!");
    }
}
void PhoneCommunication::registerCallback() { return; }
bool PhoneCommunication::isBusy() { return false; }