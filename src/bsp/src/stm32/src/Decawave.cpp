#include "Decawave.h"
#include <Task.h>
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
        Task::delay(1);
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

    dwt_writetxdata(length, buf, 0);
    dwt_writetxfctrl(length, 0, 0);

    dwt_starttx(flags);
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
    m_dwConfig = {m_channelNo,
                  DWT_PRF_64M,
                  getPreambleLength(),
                  getPACSize(),
                  getPreambleCode(),
                  getPreambleCode(),
                  1, // Use decawaves SFD sequence
                  getDWSpeed(),
                  DWT_PHRMODE_STD,
                  getSFDTimeout(64)};

    deca_selectDevice(m_spiDevice);
    dwt_configure(&m_dwConfig);
}

/********************
FUNCTIONS TO RETRIEVE DW MACROS FOR CONFIGURATIONS
********************/
uint8_t Decawave::getPreambleLength() {
    // Use biggest preamble length for given speed as higher length means higher accuracy
    // DW1000 user manual section 9.3 p.210

    switch (m_speed) {
    case UWBSpeed::SPEED_110K:
        return DWT_PLEN_256;

    case UWBSpeed::SPEED_850K:
        return DWT_PLEN_1024;

    case UWBSpeed::SPEED_6M8:
        return DWT_PLEN_4096;
    }
}

uint8_t Decawave::getPACSize() {
    // DW1000 user manual section 4.1.1 p.32

    switch (getPreambleLength()) {
    case DWT_PLEN_64:
    case DWT_PLEN_128:
        return DWT_PAC8;

    case DWT_PLEN_256:
    case DWT_PLEN_512:
        return DWT_PAC16;

    case DWT_PLEN_1024:
        return DWT_PAC32;

    default:
        return DWT_PAC64;
    }
}

uint8_t Decawave::getPreambleCode() {
    // Choose lowest code for a given channel
    // DW user manual section 10.5 p.218
    switch (m_channelNo) {
    case 4:
    case 7:
        return 17;

    default:
        return 9;
    }
}

uint8_t Decawave::getDWSpeed() {
    switch (m_speed) {
    case UWBSpeed::SPEED_110K:
        return DWT_BR_110K;

    case UWBSpeed::SPEED_850K:
        return DWT_BR_850K;

    case UWBSpeed::SPEED_6M8:
        return DWT_BR_6M8;
    }
}

uint16_t Decawave::getSFDTimeout(uint8_t sfdLength) {
    // SFDTimeout = preamble length + 1 + SFDLength - PAC size
    uint16_t preambleLength = 0;
    uint8_t pacSize = 0;

    switch (getPreambleLength()) {
    case DWT_PLEN_256:
        preambleLength = 256;
        break;

    case DWT_PLEN_1024:
        preambleLength = 1024;
        break;

    case DWT_PLEN_4096:
        preambleLength = 4096;
        break;
    }

    switch (getPACSize()) {
    case DWT_PAC8:
        pacSize = 8;
        break;

    case DWT_PAC16:
        pacSize = 16;
        break;

    case DWT_PAC32:
        pacSize = 32;
        break;

    case DWT_PAC64:
        pacSize = 64;
        break;
    }

    return preambleLength + 1 + sfdLength - pacSize;
}