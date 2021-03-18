#include "Decawave.h"
#include <DecawaveUtils.h>
#include <cstring>
#include <deca_device_api.h>

Decawave::Decawave(decaDevice_t spiDevice) :
    m_spiDevice(spiDevice), m_channelNo(2), m_speed(UWBSpeed::SPEED_110K) {}

Decawave::Decawave(decaDevice_t spiDevice, uint8_t channel, UWBSpeed speed) :
    m_spiDevice(spiDevice), m_speed(speed) {

    if (channel > 0 && channel < 8) {
        m_channelNo = channel;
        dwt_isr();
    }
}

bool Decawave::start() {
    deca_selectDevice(m_spiDevice);
    deca_setSlowRate();

    uint32_t deviceID = 0;
    uint8_t i = 0;
    // Retry to read the deviceID 10 times before abandoning
    while (deviceID != DWT_DEVICE_ID && i++ < 10) {
        deviceID = dwt_readdevid();
        // Task::delay(1);
    }

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        return false;
    }

    deca_setFastRate();

    dwt_enablegpioclocks();
    dwt_setgpiodirection(DWT_GxM0 | DWT_GxM1 | DWT_GxM2 | DWT_GxM3, 0);

    configureDW();

    setLed(DW_LED::LED_0, true);

    return true;
}

bool Decawave::setChannel(uint8_t channelNo) {
    if (channelNo < 1 || channelNo > 7) {
        return false;
    }

    m_channelNo = channelNo;
    configureDW();
    return true;
}

void Decawave::setSpeed(UWBSpeed speed) {
    m_speed = speed;
    configureDW();
}

void Decawave::setLed(DW_LED led, bool enabled) {
    uint8_t dwGPIO = 0;
    uint8_t dwGPIOValue = 0;

    switch (led) {
    case DW_LED::LED_0:
        dwGPIO = DWT_GxM0;
        if (enabled) {
            dwGPIOValue = DWT_GxP0;
        }
        break;

    case DW_LED::LED_1:
        dwGPIO = DWT_GxM1;
        if (enabled) {
            dwGPIOValue = DWT_GxP1;
        }
        break;

    case DW_LED::LED_2:
        dwGPIO = DWT_GxM2;
        if (enabled) {
            dwGPIOValue = DWT_GxP2;
        }
        break;

    case DW_LED::LED_3:
        dwGPIO = DWT_GxM3;
        if (enabled) {
            dwGPIOValue = DWT_GxP3;
        }
        break;
    }

    deca_selectDevice(m_spiDevice);
    dwt_setgpiovalue(dwGPIO, dwGPIOValue);
}

bool Decawave::transmit(uint8_t* buf, uint16_t length, uint8_t flags) {
    if (length > UWB_MAX_LENGTH) {
        return false;
    }
    deca_selectDevice(m_spiDevice);

    memcpy(m_txBuffer.data(), buf, length);

    // Send two bytes more than requested because of the DW auto-generated CRC16
    dwt_writetxdata(length + 2, m_txBuffer.data(), 0);
    dwt_writetxfctrl(length + 2, 0, 0);

    dwt_starttx(flags);

    return true;
}

bool Decawave::transmit(uint8_t* buf, uint16_t length) {
    return transmit(buf, length, DWT_START_TX_IMMEDIATE);
}

bool Decawave::transmitDelayed(uint8_t* buf, uint16_t length, uint64_t txTimestamp) {
    deca_selectDevice(m_spiDevice);
    dwt_setdelayedtrxtime(txTimestamp >> 8);

    return transmit(buf, length, DWT_START_TX_DELAYED);
}

void Decawave::configureDW() {
    uint8_t preambleLength = DecawaveUtils::getPreambleLength(m_speed);
    uint8_t pacSize = DecawaveUtils::getPACSize(preambleLength);
    uint8_t preambleCode = DecawaveUtils::getPreambleCode(m_channelNo);
    uint8_t speed = DecawaveUtils::getDWSpeed(m_speed);

    m_dwConfig = {
        m_channelNo, DWT_PRF_64M,     preambleLength,
        pacSize,     preambleCode,    preambleCode,
        1, // Use decawaves SFD sequence
        speed,       DWT_PHRMODE_STD, DecawaveUtils::getSFDTimeout(preambleLength, 64, pacSize)};

    deca_selectDevice(m_spiDevice);
    dwt_configure(&m_dwConfig);
}