#include "common/circular_buff.h"
#include <stdbool.h>
#include <stddef.h>

// TODO: Add assert instead of NULL checks
bool CircularBuff_init(CircularBuff* circularBuff, uint8_t* buffer, uint16_t size) {
    if (circularBuff == NULL || buffer == NULL) {
        return false;
    }

    circularBuff->m_data = buffer;
    circularBuff->m_size = size;
    circularBuff->m_writePos = 0;
    circularBuff->m_readPos = 0;
    circularBuff->m_isFull = false;
    return true;
}

uint16_t CircularBuff_getFreeSize(const CircularBuff* circularBuff) {
    if (circularBuff == NULL || circularBuff->m_data == NULL) {
        return 0;
    }

    if (circularBuff->m_isFull) {
        return 0;
    }

    return circularBuff->m_size - CircularBuff_getLength(circularBuff);
}

uint16_t CircularBuff_getLength(const CircularBuff* circularBuff) {
    if (circularBuff == NULL || circularBuff->m_data == NULL) {
        return 0;
    }

    if (circularBuff->m_isFull) {
        return circularBuff->m_size;
    }

    if (circularBuff->m_readPos > circularBuff->m_writePos)
        return circularBuff->m_size - (circularBuff->m_readPos - circularBuff->m_writePos);
    return circularBuff->m_writePos - circularBuff->m_readPos;
}

bool CircularBuff_isFull(const CircularBuff* circularBuff) { return circularBuff->m_isFull; }

CircularBuffRet CircularBuff_putc(CircularBuff* circularBuff, uint8_t data) {
    if (circularBuff == NULL || circularBuff->m_data == NULL) {
        return CircularBuff_Ret_NullErr;
    }
    if (CircularBuff_isFull(circularBuff)) {
        return CircularBuff_Ret_SpaceErr;
    }

    if (circularBuff->m_writePos >= circularBuff->m_size)
        circularBuff->m_writePos = 0;

    circularBuff->m_data[circularBuff->m_writePos++] = data;
    if (circularBuff->m_writePos == circularBuff->m_readPos)
        circularBuff->m_isFull = true;

    return CircularBuff_Ret_Ok;
}

CircularBuffRet CircularBuff_put(CircularBuff* circularBuff, const uint8_t* data, uint16_t length) {
    if (circularBuff == NULL || circularBuff->m_data == NULL) {
        return CircularBuff_Ret_NullErr;
    }

    if (CircularBuff_getFreeSize(circularBuff) < length) {
        return CircularBuff_Ret_SpaceErr;
    }

    for (uint16_t i = 0; i < length; i++) {
        CircularBuff_putc(circularBuff, data[i]);
    }
    return CircularBuff_Ret_Ok;
}

CircularBuffRet CircularBuff_getc(CircularBuff* circularBuff, uint8_t* data) {
    if (circularBuff == NULL || circularBuff->m_data == NULL) {
        return CircularBuff_Ret_NullErr;
    }
    if (CircularBuff_getLength(circularBuff) == 0) {
        return CircularBuff_Ret_EmptyErr;
    }

    if (circularBuff->m_readPos >= circularBuff->m_size)
        circularBuff->m_readPos = 0;

    circularBuff->m_isFull = false;

    *data = circularBuff->m_data[circularBuff->m_readPos++];
    return CircularBuff_Ret_Ok;
}

uint16_t CircularBuff_get(CircularBuff* circularBuff, uint8_t* data, uint16_t length) {
    if (circularBuff == NULL || circularBuff->m_data == NULL) {
        return 0;
    }

    uint16_t buffDataSize = CircularBuff_getLength(circularBuff);
    uint16_t copySize = length > buffDataSize ? buffDataSize : length;

    for (uint16_t i = 0; i < copySize; i++) {
        CircularBuff_getc(circularBuff, data + i);
    }

    return copySize;
}

ZeroCopyBuff CircularBuff_getZeroCopy(CircularBuff* circularBuff, uint16_t length) {
    CircularBuffRet status = CircularBuff_Ret_Ok;
    uint16_t buffDataSize = CircularBuff_getLength(circularBuff);

    if (circularBuff == NULL || circularBuff->m_data == NULL) {
        status = CircularBuff_Ret_NullErr;
    }
    if (buffDataSize == 0) {
        status = CircularBuff_Ret_EmptyErr;
    }

    uint16_t dataUntilLoop = circularBuff->m_size - circularBuff->m_readPos;

    uint16_t availableDataSize = length > buffDataSize ? buffDataSize : length;
    uint16_t copySize = availableDataSize > dataUntilLoop ? dataUntilLoop : availableDataSize;

    ZeroCopyBuff buff = {.data = circularBuff->m_data + circularBuff->m_readPos,
                         .length = copySize,
                         .status = status};

    // Update the state
    circularBuff->m_readPos = copySize > dataUntilLoop ? 0 : circularBuff->m_readPos + copySize;
    if (length > 0) {
        circularBuff->m_isFull = false;
    }

    return buff;
}

bool CircularBuff_clear(CircularBuff* circularBuff) {
    if (circularBuff == NULL || circularBuff->m_data == NULL) {
        return false;
    }

    circularBuff->m_readPos = 0;
    circularBuff->m_writePos = 0;
    circularBuff->m_isFull = false;
    return true;
}
