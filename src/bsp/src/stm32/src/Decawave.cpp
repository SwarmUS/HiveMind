#include "Decawave.h"
#include <DecawaveUtils.h>
#include <FreeRTOS.h>
#include <Task.h>
#include <cstring>
#include <deca_device_api.h>
#include <deca_regs.h>
#include <task.h>

void rxCallback(const dwt_cb_data_t* callbackData, void* context) {
    memcpy(&(static_cast<Decawave*>(context)->m_callbackData), callbackData, sizeof(dwt_cb_data_t));

    BaseType_t taskWoken = pdFALSE;

    if (static_cast<Decawave*>(context)->m_rxTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(static_cast<Decawave*>(context)->m_rxTaskHandle, &taskWoken);
    }

    static_cast<Decawave*>(context)->m_rxTaskHandle = NULL;
    portYIELD_FROM_ISR(taskWoken);
}

void isrCallback(void* context) {
    deca_selectDevice(static_cast<Decawave*>(context)->m_spiDevice);
    dwt_isr();
}

void rxAsyncTask(void* context) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        static_cast<Decawave*>(context)->retrieveRxFrame(
            static_cast<Decawave*>(context)->m_rxFrame);
    }
}

Decawave::Decawave(decaDevice_t spiDevice) :
    m_spiDevice(spiDevice),
    m_channelNo(2),
    m_speed(UWBSpeed::SPEED_110K),
    m_rxAsyncTask("dw_rx_task", tskIDLE_PRIORITY + 10, rxAsyncTask, this) {}

Decawave::Decawave(decaDevice_t spiDevice, uint8_t channel, UWBSpeed speed) :
    m_spiDevice(spiDevice),
    m_speed(speed),
    m_rxAsyncTask("dw_rx_task", tskIDLE_PRIORITY + 10, rxAsyncTask, this) {

    if (channel > 0 && channel < 8) {
        m_channelNo = channel;
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

    deca_selectDevice(m_spiDevice);
    deca_setISRCallback(m_spiDevice, isrCallback, this);
    dwt_setcallbacks(rxCallback, rxCallback, rxCallback, rxCallback, this);

    deca_setFastRate();

    dwt_softreset();
    configureDW();

    dwt_enablegpioclocks();
    dwt_setgpiodirection(DWT_GxM0 | DWT_GxM1 | DWT_GxM2 | DWT_GxM3, 0);

    dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE |
                         DWT_INT_RFSL | DWT_INT_SFDT,
                     1);

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

void Decawave::receive(UWBRxFrame& frame, uint16_t timeoutUs) {
    m_rxTaskHandle = xTaskGetCurrentTaskHandle();

    frame.m_status = UWBRxStatus::ONGOING;

    dwt_setrxtimeout(timeoutUs);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    retrieveRxFrame(&frame);
}

void Decawave::receiveAsync(UWBRxFrame& frame, uint16_t timeoutUs) {
    m_rxFrame = &frame;
    m_rxTaskHandle = m_rxAsyncTask.getTaskHandle();

    frame.m_status = UWBRxStatus::ONGOING;

    dwt_setrxtimeout(timeoutUs);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
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

void Decawave::retrieveRxFrame(UWBRxFrame* frame) {
    if (frame == nullptr) {
        return;
    }

    deca_selectDevice(m_spiDevice);

    frame->m_statusReg = m_callbackData.status;
    frame->m_length = m_callbackData.datalength;

    // Frame was properly received. Read all relevant data from DW
    if (m_callbackData.datalength > 0) {
        frame->m_status = UWBRxStatus::FINISHED;

        // Read the frame into memory without the CRC16 located at the end of the frame
        dwt_readrxdata(frame->m_rxBuffer.data(), m_callbackData.datalength - 2, 0);
        dwt_readrxtimestamp((uint8_t*)(&frame->m_rxTimestamp));
        return;
    }

    // Frame was not received, parse status reg to find the error type
    if ((m_callbackData.status & SYS_STATUS_ALL_RX_TO) != 0U) {
        frame->m_status = UWBRxStatus::TIMEOUT;
    } else if ((m_callbackData.status & SYS_STATUS_ALL_RX_ERR) != 0U) {
        frame->m_status = UWBRxStatus::ERROR;
    }
}