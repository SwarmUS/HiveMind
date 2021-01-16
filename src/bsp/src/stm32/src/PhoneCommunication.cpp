#include "PhoneCommunication.h"
#include "hal/hal.h"
#include "hal/uart_phone.h"
#include <cstdio>
#include <cstring>

void PhoneCommunication::sendBytes(const uint8_t* bytes, uint16_t length) {
    *txBuffer = length;
    uint32_t crc = Hal_calculateCRC32(bytes, length);
    memcpy((txBuffer + sizeof(length)), &crc, sizeof(crc));
    memcpy((txBuffer + sizeof(length) + sizeof(crc)), bytes, length);

    bool ret = UartPhone_transmitBuffer(txBuffer, length + sizeof(length) + sizeof(crc));
    if (!ret) {
        printf("Could not transmit to phone!");
    }
}
void PhoneCommunication::registerCallback() { return; }
bool PhoneCommunication::isBusy() { return false; }