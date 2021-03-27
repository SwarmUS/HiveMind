#include "Decawave.h"
#include <Task.h>
#include <cpp-common/CppUtils.h>
#include <cstring>
#include <deca_device_api.h>
#include <deca_regs.h>
#include "hal/hal_gpio.h"

void Decawave::rxCallback(const dwt_cb_data_t* callbackData, void* context) {
    memcpy(&(static_cast<Decawave*>(context)->m_callbackData), callbackData, sizeof(dwt_cb_data_t));
    //TEMP
    uint8_t myBuff[15];
    dwt_readrxdata(&myBuff[0],callbackData->datalength,0);

    BaseType_t taskWoken = pdFALSE;
    volatile auto* test = static_cast<Decawave*>(context);
    (void) test;

    if (static_cast<Decawave*>(context)->m_rxTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(static_cast<Decawave*>(context)->m_rxTaskHandle,&taskWoken);
    }

    static_cast<Decawave*>(context)->m_rxTaskHandle = NULL;
    portYIELD_FROM_ISR(taskWoken);
}

void Decawave::isrCallback(void* context) {
    deca_selectDevice(static_cast<Decawave*>(context)->m_spiDevice);
    dwt_isr();
}

void Decawave::rxAsyncTask(void* context) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        static_cast<Decawave*>(context)->retrieveRxFrame(
            static_cast<Decawave*>(context)->m_rxFrame);
    }
}

Decawave::Decawave(decaDevice_t spiDevice) :
    m_spiDevice(spiDevice),
    m_channel(UWBChannel::CHANNEL_2),
    m_speed(UWBSpeed::SPEED_110K),
    m_rxAsyncTask("dw_rx_task", tskIDLE_PRIORITY + 10, rxAsyncTask, this) {}

Decawave::Decawave(decaDevice_t spiDevice, UWBChannel channel, UWBSpeed speed) :
    m_spiDevice(spiDevice),
    m_channel(channel),
    m_speed(speed),
    m_rxAsyncTask("dw_rx_task", tskIDLE_PRIORITY + 10, rxAsyncTask, this) {}

bool Decawave::init() {
    deca_selectDevice(m_spiDevice);
    deca_setSlowRate();
    uint32_t deviceID = 0;
    uint8_t i = 0;

    // Retry to read the deviceID 10 times before abandoning
    do {
        deviceID = dwt_readdevid();
        Task::delay(1);
    } while (deviceID != DWT_DEVICE_ID && 10 > i++);

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

    //temp
    setResetSendBtnCallback(Decawave::interruptBtnSendCallback, this);
    setResetReceiveBtnCallback(Decawave::interruptBtnReceiveCallback, this);
    m_rxAsyncTask.start();

    return true;
}

bool Decawave::setChannel(UWBChannel channel) {
    m_channel = channel;
    configureDW();
    return true;
}

void Decawave::setSpeed(UWBSpeed speed) {
    m_speed = speed;
    configureDW();
}

void Decawave::setLed(DW_LED led, bool enabled) {
    deca_selectDevice(m_spiDevice);

    if (enabled) {
        dwt_setgpiovalue(static_cast<uint32>(as_integer(led)),
                         static_cast<uint32>(as_integer(led) >> 4));
    } else {
        dwt_setgpiovalue(static_cast<uint32>(as_integer(led)), 0);
    }
}

void Decawave::receiveInternal(UWBRxFrame& frame,
                               uint16_t timeoutUs,
                               uint8_t flags,
                               bool rxStarted) {
    deca_selectDevice(m_spiDevice);
    m_rxTaskHandle = xTaskGetCurrentTaskHandle();


    frame.m_status = UWBRxStatus::ONGOING;

    if (!rxStarted) {
        dwt_setrxtimeout(timeoutUs);
        dwt_rxenable(flags);
    }

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    retrieveRxFrame(&frame);
}

void Decawave::receiveAsyncInternal(UWBRxFrame& frame,
                                    uint16_t timeoutUs,
                                    uint8_t flags,
                                    bool rxStarted) {
    m_rxFrame = &frame;
    m_rxTaskHandle = m_rxAsyncTask.getTaskHandle();

    deca_selectDevice(m_spiDevice);

    frame.m_status = UWBRxStatus::ONGOING;

    if (!rxStarted) {
        dwt_setrxtimeout(timeoutUs);
        dwt_rxenable(flags);
    }
}

void Decawave::receive(UWBRxFrame& frame, uint16_t timeoutUs) {
    receiveInternal(frame, timeoutUs, DWT_START_RX_IMMEDIATE);
}

void Decawave::receiveDelayed(UWBRxFrame& frame, uint16_t timeoutUs, uint64_t rxStartTime) {
    deca_selectDevice(m_spiDevice);
    dwt_setdelayedtrxtime(rxStartTime >> 8);

    receiveInternal(frame, timeoutUs, rxStartTime);
}

void Decawave::receiveAsync(UWBRxFrame& frame, uint16_t timeoutUs) {
    receiveAsyncInternal(frame, timeoutUs, DWT_START_TX_IMMEDIATE);
}

void Decawave::receiveAsyncDelayed(UWBRxFrame& frame, uint16_t timeoutUs, uint64_t rxStartTime) {
    deca_selectDevice(m_spiDevice);
    dwt_setdelayedtrxtime(rxStartTime >> 8);

    receiveAsyncInternal(frame, timeoutUs, rxStartTime);
}

bool Decawave::transmitInternal(uint8_t* buf, uint16_t length, uint8_t flags) {
    if (length > (UWB_MAX_LENGTH - UWB_CRC_LENGTH)) {
        return false;
    }
    deca_selectDevice(m_spiDevice);

    memcpy(m_txBuffer.data(), buf, length);

    // Send two bytes more than requested because of the DW auto-generated CRC16
    dwt_writetxdata(length + UWB_CRC_LENGTH, m_txBuffer.data(), 0);
    dwt_writetxfctrl(length + UWB_CRC_LENGTH, 0, 0);

    dwt_starttx(flags);

    return true;
}

bool Decawave::transmit(uint8_t* buf, uint16_t length) {
    return transmitInternal(buf, length, DWT_START_TX_IMMEDIATE);
}

bool Decawave::transmitDelayed(uint8_t* buf, uint16_t length, uint64_t txTimestamp) {
    deca_selectDevice(m_spiDevice);
    dwt_setdelayedtrxtime(txTimestamp >> 8);

    return transmitInternal(buf, length, DWT_START_TX_DELAYED);
}

bool Decawave::transmitAndReceive(uint8_t* buf,
                                  uint16_t length,
                                  uint32_t rxAfterTxTimeUs,
                                  UWBRxFrame& frame,
                                  uint16_t rxTimeoutUs) {
    deca_selectDevice(m_spiDevice);
    dwt_setrxaftertxdelay(rxAfterTxTimeUs);
    dwt_setrxtimeout(rxTimeoutUs);

    if (!transmitInternal(buf, length, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED)) {
        return false;
    }

    receiveInternal(frame, 0, 0, true);
    return true;
}

bool Decawave::transmitDelayedAndReceive(uint8_t* buf,
                                            uint16_t length,
                                         uint64_t txTimestamp,
                                         uint32_t rxAfterTxTimeUs,
                                         UWBRxFrame& frame,
                                         uint16_t rxTimeoutUs) {
    deca_selectDevice(m_spiDevice);
    dwt_setrxaftertxdelay(rxAfterTxTimeUs);
    dwt_setdelayedtrxtime(txTimestamp >> 8);
    dwt_setrxtimeout(rxTimeoutUs);

    if (!transmitInternal(buf, length, DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED)) {
        return false;
    }

    receiveInternal(frame, 0, 0, true);
    return true;
}
bool Decawave::transmitAndReceiveDelayed(uint8_t* buf,
                                         uint16_t length,
                                         uint32_t rxStartDelayUS,
                                         UWBRxFrame& frame,
                                         uint16_t rxTimeoutUs){
    deca_selectDevice(m_spiDevice);
    dwt_setrxaftertxdelay(rxStartDelayUS);
    dwt_setrxtimeout(rxTimeoutUs);

    if(!transmitInternal(buf, length, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED)) {
        return false;
    }
    receiveInternal(frame, 0, 0, true);
    return true;


}

void Decawave::configureDW() {
    uint8_t preambleLength = DecawaveUtils::getPreambleLength(m_speed);
    uint8_t pacSize = DecawaveUtils::getPACSize(preambleLength);
    uint8_t preambleCode = DecawaveUtils::getPreambleCode(m_channel);

    m_dwConfig = {static_cast<uint8>(as_integer(m_channel)),
                  DWT_PRF_64M,
                  preambleLength,
                  pacSize,
                  preambleCode,
                  preambleCode,
                  1, // Use decawaves SFD sequence
                  static_cast<uint8>(as_integer(m_speed)),
                  DWT_PHRMODE_STD,
                  DecawaveUtils::getSFDTimeout(preambleLength, 64, pacSize)};

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
        dwt_readrxdata(frame->m_rxBuffer.data(), m_callbackData.datalength - UWB_CRC_LENGTH, 0);
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

void Decawave::setTxAntennaDLY(uint16 delay){
    deca_selectDevice(m_spiDevice);
    dwt_settxantennadelay(delay);
}

void Decawave::setRxAntennaDLY(uint16 delay){
    deca_selectDevice(m_spiDevice);
    dwt_setrxantennadelay(delay);
}

void Decawave::getTxTimestamp(uint64_t *txTimestamp){
    deca_selectDevice(m_spiDevice);
    dwt_readtxtimestamp(reinterpret_cast<uint8*>(txTimestamp));
}

DW_STATE Decawave::getState(){
    return m_state;
}
void Decawave::setState(DW_STATE state){
    m_state = state;
}

uint16_t Decawave::getRxAntennaDLY(){
    deca_selectDevice(m_spiDevice);
    return dwt_read16bitoffsetreg(LDE_IF_ID, LDE_RXANTD_OFFSET);

}
uint16_t Decawave::getTxAntennaDLY(){
    deca_selectDevice(m_spiDevice);
    return dwt_read16bitoffsetreg(TX_ANTD_ID, TX_ANTD_OFFSET);
}

void Decawave::resetBtnSendTWRCallback(){
    deca_selectDevice(m_spiDevice);
    setState(DW_STATE::SEND_CALIB);
    setLed(DW_LED::LED_3, false);
    setLed(DW_LED::LED_2, true);
}
void Decawave::resetBtnReceiveTWRCallback(){
    setState(DW_STATE::RECEIVE_CALIB);
    setLed(DW_LED::LED_3, true);
    setLed(DW_LED::LED_2, false);
}

void Decawave::interruptBtnSendCallback(void* context) {
    Decawave* deca = (Decawave*)context;
    deca->resetBtnSendTWRCallback();
}

void Decawave::interruptBtnReceiveCallback(void* context) {
    Decawave* deca = (Decawave*)context;
    deca->resetBtnReceiveTWRCallback();
}