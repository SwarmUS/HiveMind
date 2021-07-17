#include "TestUSB.h"
#include <cstdio>
#include <hal/usb.h>

void TestUSB::runTests() {
    setup();

    test01_UART();
    test02_USB();
}

void TestUSB::test01_UART() {
    // Connect to COM port from PC
    printf("Hello World!\n\r");
    HAL_Delay(2000);
}

void TestUSB::test02_USB() {
    HAL_Delay(2000);
    // Disconnect USB from PC
    if (usb_isConnected()) {
        while (true) {
        }
    }

    HAL_Delay(2000);
    // Connect USB to PC without opening VCP
    if (usb_isConnected()) {
        while (true) {
        }
    }

    HAL_Delay(2000);
    if (USB_DEVICE.dev_state != USBD_STATE_CONFIGURED) {
        while (true) {
        }
    }

    HAL_Delay(2000);
    // Open VCP
    if (!usb_isConnected()) {
        while (true) {
        }
    }

    uint8_t data[13] = "Hello World!";
    usb_sendData(data, sizeof(data));

    // Send data on VCP to continue
    usb_setRxCallback(rxCallback, this);
    while (!m_receivedData) {
        HAL_Delay(100);
    }

    // Verify received data
    (void)cbuffUsb;
    HAL_Delay(100); // Put breakpoint here
}

void TestUSB::rxCallback(void* context, uint8_t* buffer, uint32_t length) {
    CircularBuff_put(&cbuffUsb, buffer, length);
    static_cast<TestUSB*>(context)->m_receivedData = true;
}

void TestUSB::setup() { usb_init(); }
