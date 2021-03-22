#include "hal/usb.h"
#include "Task.h"
#include "usbd_cdc_if.h"
#include <USB.h>

#define USB_RxBUFFER_MAX_SIZE 2048
void USB::interruptRxCallback(void* context, uint8_t* buffer, uint32_t length) {
    USB* usb = (USB*)context;
    usb->receiveItCallback(buffer, length);
}

bool USB::send(const uint8_t* buffer, uint16_t length) {
    if (buffer == nullptr) {
        m_logger.log(LogLevel::Warn, "Invalid parameters for USB::send");
        return false;
    }
    while (!usb_isConnected()) {
        Task::delay(100);
    }

    int position = 0;
    while (length - position > USB_RxBUFFER_MAX_SIZE) {
        usb_sendData(const_cast<uint8_t*>(buffer + position), USB_RxBUFFER_MAX_SIZE);
        position += USB_RxBUFFER_MAX_SIZE;
    }

    USB_StatusTypeDef out = usb_sendData(const_cast<uint8_t*>(buffer + position), length);

    if (out != USB_OK) {
        m_logger.log(LogLevel::Warn, "USB_Send_Data was not able to send the data");
        return false;
    }

    return true;
}

bool USB::receive(uint8_t* buffer, uint16_t length) {
    if (buffer == nullptr || length > CBUFF_USB_DATA_SIZE) {
        m_logger.log(LogLevel::Warn, "Invalid parameters for USB::Receive");
        return false;
    }

    while (CircularBuff_getLength(&cbuffUsb) < length) {
        // Gets notified everytime a new packet is appended to cbuffUsb
        ulTaskNotifyTake(pdTRUE, 500);
    }
    m_receivingTaskHandle = NULL;

    CircularBuff_get(&cbuffUsb, buffer, length);

    return true;
}

bool USB::isConnected() { return usb_isConnected(); }

void USB::receiveItCallback(uint8_t* buf, uint32_t len) {
    if (CircularBuff_getLength(&cbuffUsb) + len > CBUFF_USB_DATA_SIZE) {
        // TODO should notify the user of an error
        CircularBuff_clear(&cbuffUsb);
        m_logger.log(LogLevel::Error, "USB buffer full, erasing circular buffer");
        return;
    }

    CircularBuff_put(&cbuffUsb, buf, len);

    BaseType_t toYield = pdFALSE;
    if (m_receivingTaskHandle != NULL) {
        vTaskNotifyGiveIndexedFromISR(m_receivingTaskHandle, 0, &toYield);
    }

    portYIELD_FROM_ISR(toYield);
}

USB::USB(ILogger& logger) : m_logger(logger) { usb_setRxCallback(USB::interruptRxCallback, this); }
