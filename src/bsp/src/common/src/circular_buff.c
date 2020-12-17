#include "common/circular_buff.h"
#include <stdbool.h>
#include <stddef.h>

bool CircularBuff_init(CircularBuff* circularBuff, uint8_t* buffer, uint16_t size) {
    if (circularBuff == NULL || buffer == NULL) {
        return false;
    }
    circularBuff->data = buffer;
    circularBuff->size = size;
    circularBuff->writePos = 0;
    circularBuff->readPos = 0;
    circularBuff->isFull = false;
    return true;
}

uint16_t CircularBuff_getFreeSize(const CircularBuff* circularBuff) {
    if (circularBuff == NULL || circularBuff->data == NULL) {
        return 0;
    }

    if (circularBuff->isFull) {
        return 0;
    }

    return circularBuff->size - CircularBuff_getLength(circularBuff);
}

uint16_t CircularBuff_getLength(const CircularBuff* circularBuff) {
    if (circularBuff == NULL || circularBuff->data == NULL) {
        return 0;
    }

    if (circularBuff->isFull) {
        return circularBuff->size;
    }

    if (circularBuff->readPos > circularBuff->writePos)
        return circularBuff->size - (circularBuff->readPos - circularBuff->writePos);
    return circularBuff->writePos - circularBuff->readPos;
}

bool CircularBuff_isFull(const CircularBuff* circularBuff) { return circularBuff->isFull; }

CircularBuffRet CircularBuff_putc(CircularBuff* circularBuff, uint8_t data) {
    if (circularBuff == NULL || circularBuff->data == NULL) {
        return CircularBuff_Ret_NullErr;
    }
    if (CircularBuff_isFull(circularBuff)) {
        return CircularBuff_Ret_SpaceErr;
    }

    if (circularBuff->writePos >= circularBuff->size)
        circularBuff->writePos = 0;

    circularBuff->data[circularBuff->writePos++] = data;
    return CircularBuff_Ret_Ok;
}

CircularBuffRet CircularBuff_put(CircularBuff* circularBuff, const uint8_t* data, uint16_t length) {
    if (circularBuff == NULL || circularBuff->data) {
        return CircularBuff_Ret_NullErr;
    }

    if (CircularBuff_getFreeSize(circularBuff) < length) {
        return CircularBuff_Ret_SpaceErr;
    }

    for (uint16_t i = 0; i <= length; i++) {
        CircularBuff_putc(circularBuff, data[i]);
    }
    return CircularBuff_Ret_Ok;
}

CircularBuffRet CircularBuff_readc(CircularBuff* circularBuff, uint8_t* data) {
    if (circularBuff == NULL || circularBuff->data == NULL) {
        return CircularBuff_Ret_NullErr;
    }
    if (CircularBuff_getLength(circularBuff) == 0) {
        return CircularBuff_Ret_EmptyErr;
    }
    if (circularBuff->readPos >= circularBuff->size)
        circularBuff->readPos = 0;

    *data = circularBuff->data[circularBuff->readPos++];
    return CircularBuff_Ret_Ok;
}

uint16_t CircularBuff_read(CircularBuff* circularBuff, uint8_t* data, uint16_t length) {
    if (circularBuff == NULL || circularBuff->data == NULL) {
        return 0;
    }

    uint16_t buff_data_size = CircularBuff_getLength(circularBuff);
    uint16_t copy_size = length >= buff_data_size ? buff_data_size : length;

    for (uint16_t i = 0; i < copy_size; i++) {
        CircularBuff_readc(circularBuff, data + i);
    }

    return copy_size;
}

bool CircularBuff_clear(CircularBuff* circularBuff) {
    if (circularBuff == NULL || circularBuff->data == NULL) {
        return false;
    }

    circularBuff->readPos = 0;
    circularBuff->writePos = 0;
    return true;
}
