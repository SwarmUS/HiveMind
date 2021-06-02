#include "interloc/Decawave.h"
#include <Task.h>
#include <cpp-common/CppUtils.h>
#include <cstring>
#include <deca_device_api.h>
#include <deca_regs.h>

void Decawave::rxCallback(const dwt_cb_data_t* callbackData, void* context) {
    memcpy(&(static_cast<Decawave*>(context)->m_callbackData), callbackData, sizeof(dwt_cb_data_t));
    BaseType_t taskWoken = pdFALSE;

    if (static_cast<Decawave*>(context)->m_trxTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(static_cast<Decawave*>(context)->m_trxTaskHandle, &taskWoken);
    }

    static_cast<Decawave*>(context)->m_trxTaskHandle = NULL;
    portYIELD_FROM_ISR(taskWoken);
}

void Decawave::txCallback(const dwt_cb_data_t* callbackData, void* context) {
    BaseType_t taskWoken = pdFALSE;

    (void)callbackData;

    if (static_cast<Decawave*>(context)->m_trxTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(static_cast<Decawave*>(context)->m_trxTaskHandle, &taskWoken);
    }

    static_cast<Decawave*>(context)->m_trxTaskHandle = NULL;
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
    m_channel(UWBChannel::DW_CHANNEL),
    m_speed(UWBSpeed::DW_SPEED),
    m_rxAsyncTask("dw_rx_task", tskIDLE_PRIORITY + 10, rxAsyncTask, this),
    m_isReady(false) {}

bool Decawave::init() {
    if (!deca_isPresent(m_spiDevice)) {
        m_isReady = false;
        return false;
    }

    deca_selectDevice(m_spiDevice);
    deca_setSlowRate(m_spiDevice);
    uint32_t deviceID = 0;
    uint8_t i = 0;

    // Retry to read the deviceID 10 times before abandoning
    do {
        deviceID = dwt_readdevid();
        Task::delay(1);
    } while (deviceID != DWT_DEVICE_ID && 10 > i++);

    if (deviceID != DWT_DEVICE_ID) {
        m_isReady = false;
        return false;
    }

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        return false;
    }

    deca_selectDevice(m_spiDevice);
    deca_setISRCallback(m_spiDevice, isrCallback, this);
    dwt_setcallbacks(txCallback, rxCallback, rxCallback, rxCallback, this);

    deca_setFastRate(m_spiDevice);

    dwt_softreset();
    configureDW();

    setRxAntennaDLY(DEFAULT_RX_ANT_DLY);
    setTxAntennaDLY(DEFAULT_TX_ANT_DLY);

    setState(DW_STATE::CONFIGURED);

    dwt_enablegpioclocks();
    dwt_setgpiodirection(DWT_GxM0 | DWT_GxM1 | DWT_GxM2 | DWT_GxM3, 0);

    dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE |
                         DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_TFRS,
                     1);

    setLed(DW_LED::LED_0, true);

    uint32_t sysCfgReg = dwt_read32bitoffsetreg(SYS_CFG_ID, 0);
    dwt_write32bitoffsetreg(SYS_CFG_ID, 0, sysCfgReg | SYS_CFG_RXAUTR);

    m_rxAsyncTask.start();

    m_isReady = true;
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
    m_trxTaskHandle = xTaskGetCurrentTaskHandle();

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
    m_trxTaskHandle = m_rxAsyncTask.getTaskHandle();

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

    receiveInternal(frame, timeoutUs, DWT_START_RX_DELAYED);
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

    int txStatus = dwt_starttx(flags);

    if (txStatus == 0) {
        // wait for the end of the transmission
        m_trxTaskHandle = xTaskGetCurrentTaskHandle();
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    } else {
        //        volatile uint64_t currentTime = getSysTime();
        //        (void)currentTime;
        //        // TODO: For debugging. Remove and handle correctly in real application
        //        while (true) {
        //        }
    }

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
                                         uint16_t rxTimeoutUs) {
    deca_selectDevice(m_spiDevice);
    dwt_setrxaftertxdelay(rxStartDelayUS);
    dwt_setrxtimeout(rxTimeoutUs);

    if (!transmitInternal(buf, length, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED)) {
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

    // Frame was not received, parse status reg to find the error type
    if ((m_callbackData.status & SYS_STATUS_ALL_RX_TO) != 0U) {
        frame->m_status = UWBRxStatus::TIMEOUT;
        return;
    }

    if ((m_callbackData.status & SYS_STATUS_ALL_RX_ERR) != 0U) {
        frame->m_status = UWBRxStatus::ERROR;
        return;
    }

    frame->m_statusReg = m_callbackData.status;
    frame->m_length = m_callbackData.datalength;
    frame->m_status = UWBRxStatus::FINISHED;

    // Read the frame into memory without the CRC16 located at the end of the frame
    dwt_readrxdata(frame->m_rxBuffer.data(), m_callbackData.datalength - UWB_CRC_LENGTH, 0);
    getRxTimestamp(&frame->m_rxTimestamp);

    dwt_readfromdevice(RX_TTCKO_ID, 4, 1, &(frame->m_sfdAngleRegister));
    // Read information needed for phase calculation
    uint16_t firstPathIdx = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET);
    // Read one extra byte as readaccdata() returns a dummy byte
    dwt_readaccdata(frame->m_firstPathAccumulator, 5, firstPathIdx);
}

void Decawave::setSyncMode(DW_SYNC_MODE syncMode) {
    deca_selectDevice(m_spiDevice);
    uint16_t regValue = 0;
    constexpr uint16_t syncWaitTime = 33; // should have waitTime%4 = 1 (DW1000 user manual p.56)

    switch (syncMode) {

    case DW_SYNC_MODE::OSTR:
        regValue = dwt_read16bitoffsetreg(EXT_SYNC_ID, EC_CTRL_OFFSET);
        regValue &= EC_CTRL_WAIT_MASK; // Clear previous wait value
        regValue |= EC_CTRL_OSTRM;
        regValue |= (((syncWaitTime)&0xff) << 3);
        dwt_write16bitoffsetreg(EXT_SYNC_ID, EC_CTRL_OFFSET, regValue);
        break;

    case DW_SYNC_MODE::OFF:
        regValue = dwt_read16bitoffsetreg(EXT_SYNC_ID, EC_CTRL_OFFSET);
        regValue &= EC_CTRL_WAIT_MASK; // Clear previous wait value
        dwt_write16bitoffsetreg(EXT_SYNC_ID, EC_CTRL_OFFSET, regValue);
        break;
    }
}

void Decawave::setTxAntennaDLY(uint16 delay) {
    m_txAntennaDelayDTU = delay;

    deca_selectDevice(m_spiDevice);
    dwt_settxantennadelay(delay);
}

void Decawave::setRxAntennaDLY(uint16 delay) {
    m_rxAntennaDelayDTU = delay;

    deca_selectDevice(m_spiDevice);
    dwt_setrxantennadelay(delay);
}

uint16_t Decawave::getRxAntennaDLY() const { return m_rxAntennaDelayDTU; }

uint16_t Decawave::getTxAntennaDLY() const { return m_rxAntennaDelayDTU; }

void Decawave::getTxTimestamp(uint64_t* txTimestamp) {
    deca_selectDevice(m_spiDevice);
    dwt_readtxtimestamp((uint8_t*)txTimestamp);
}

void Decawave::getRxTimestamp(uint64_t* rxTimestamp) {
    deca_selectDevice(m_spiDevice);
    dwt_readrxtimestamp((uint8_t*)rxTimestamp);
}

uint64_t Decawave::getSysTime() {
    uint64_t ts = 0;
    uint8_t time[5];
    deca_selectDevice(m_spiDevice);
    dwt_readsystime(time);

    for (int i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= time[i];
    }

    return ts;
}

uint64_t Decawave::getTxTimestampFromDelayedTime(uint64_t txTime) const {
    // The DW100 delayed transmit has a resolution of 512 DTUs (so lower 9 bits are masked off the
    // get the time at which it will really be sent)
    return (txTime & 0xFFFFFFFE00UL) + getTxAntennaDLY();
}

DW_STATE Decawave::getState() { return m_state; }

void Decawave::setState(DW_STATE state) { m_state = state; }
