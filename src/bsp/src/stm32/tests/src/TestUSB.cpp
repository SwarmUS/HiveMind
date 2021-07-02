#include "TestUSB.h"
#include "../../include/USB.h"
#include <cstdio>
#include <hal/usb.h>

void TestUSB::runTests() {
    setup();

    test01_UART();
    test02_USB();
}

void TestUSB::test01_UART() {
    // Connect to COM port from PC
    printf("Hello World!");
}

void TestUSB::test02_USB() {
    // Disconnect USB from PC
    if (usb_isConnected()) {
        while (true) {
        }
    }

    // Connect USB to PC without opening VCP
    if (usb_isConnected()) {
        while (true) {
        }
    }

    if (USB_DEVICE.dev_state != USBD_STATE_CONFIGURED) {
        while (true) {
        }
    }

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
}

void TestUSB::rxCallback(void* context, uint8_t* buffer, uint32_t length) {
    CircularBuff_put(&cbuffUsb, buffer, length);
    static_cast<TestUSB*>(context)->m_receivedData = true;
}

void TestUSB::setup() { usb_init(); }
