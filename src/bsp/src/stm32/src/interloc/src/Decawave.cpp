#include "interloc/Decawave.h"
#include <Task.h>
#include <cpp-common/CppUtils.h>
#include <cstring>
#include <deca_device_api.h>
#include <deca_regs.h>

void Decawave::rxCallback(const dwt_cb_data_t* callbackData, void* context) {
    memcpy(&(static_cast<Decawave*>(context)->m_callbackData), callbackData, sizeof(dwt_cb_data_t));
    BaseType_t taskWoken = pdFALSE;

    // Frame was not received, parse status reg to find the error type
    if ((callbackData->status & SYS_STATUS_ALL_RX_TO) != 0U) {
        static_cast<Decawave*>(context)->m_rxStatus = UWBRxStatus::TIMEOUT;
    } else if ((callbackData->status & SYS_STATUS_ALL_RX_ERR) != 0U) {
        static_cast<Decawave*>(context)->m_rxStatus = UWBRxStatus::ERROR;
    } else {
        static_cast<Decawave*>(context)->m_rxStatus = UWBRxStatus::FINISHED;
    }

    if (static_cast<Decawave*>(context)->m_trxTaskHandle != nullptr) {
        vTaskNotifyGiveFromISR(static_cast<Decawave*>(context)->m_trxTaskHandle, &taskWoken);
    }

    static_cast<Decawave*>(context)->m_trxTaskHandle = nullptr;
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

Decawave::Decawave(decaDevice_t spiDevice) :
    m_spiDevice(spiDevice),
    m_channel(UWBChannel::DW_CHANNEL),
    m_speed(UWBSpeed::DW_SPEED),
    m_rxStatus(UWBRxStatus::FINISHED),
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

    enableAccumulatorClock();

    dwt_setgpiodirection(DWT_GxM0 | DWT_GxM1 | DWT_GxM2 | DWT_GxM3, 0);

    dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE |
                         DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_TFRS,
                     1);

    setLed(DW_LED::LED_2, true);

    uint32_t sysCfgReg = dwt_read32bitoffsetreg(SYS_CFG_ID, 0);
    dwt_write32bitoffsetreg(SYS_CFG_ID, 0, sysCfgReg | SYS_CFG_RXAUTR);

    m_isReady = true;
    return true;
}
void Decawave::enableAccumulatorClock() const {
    uint8 reg[2];
    dwt_readfromdevice(PMSC_ID, PMSC_CTRL0_OFFSET, 2, reg);
    reg[0] = 0x48 | (reg[0] & 0xb3);
    reg[1] = 0x80 | reg[1];
    dwt_writetodevice(PMSC_ID, PMSC_CTRL0_OFFSET, 1, &reg[0]);
    dwt_writetodevice(PMSC_ID, 0x1, 1, &reg[1]);
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

UWBRxStatus Decawave::receiveInternal(uint16_t timeoutUs, uint8_t flags, bool rxStarted) {
    deca_selectDevice(m_spiDevice);
    m_rxStatus = UWBRxStatus::ONGOING;

    if (!rxStarted) {
        dwt_setrxtimeout(timeoutUs);
        dwt_rxenable(flags);
    }

    return awaitRx();
}

void Decawave::receiveAsyncInternal(uint16_t timeoutUs, uint8_t flags, bool rxStarted) {
    m_trxTaskHandle = nullptr;
    m_rxStatus = UWBRxStatus::ONGOING;

    deca_selectDevice(m_spiDevice);

    if (!rxStarted) {
        dwt_setrxtimeout(timeoutUs);
        dwt_rxenable(flags);
    }
}

UWBRxStatus Decawave::receive(uint16_t timeoutUs) {
    return receiveInternal(timeoutUs, DWT_START_RX_IMMEDIATE);
}

UWBRxStatus Decawave::receiveDelayed(uint16_t timeoutUs, uint64_t rxStartTime) {
    deca_selectDevice(m_spiDevice);
    dwt_setdelayedtrxtime(rxStartTime >> 8);

    return receiveInternal(timeoutUs, DWT_START_RX_DELAYED);
}

void Decawave::receiveAsync(uint16_t timeoutUs) {
    receiveAsyncInternal(timeoutUs, DWT_START_RX_IMMEDIATE);
}

void Decawave::receiveAsyncDelayed(uint16_t timeoutUs, uint64_t rxStartTime) {
    deca_selectDevice(m_spiDevice);
    dwt_setdelayedtrxtime(rxStartTime >> 8);

    receiveAsyncInternal(timeoutUs, DWT_START_RX_DELAYED);
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
        ulTaskNotifyTake(pdTRUE, 100); // Avoid blocking here if something goes wrong
        m_trxTaskHandle = nullptr;
    } else {
        //        volatile uint64_t currentTime = getSysTime();
        //        (void)currentTime;
        // TODO: For debugging. Remove and handle correctly in real application
        //        while (true) {
        //        }
        return false;
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

std::optional<UWBRxStatus> Decawave::transmitAndReceive(uint8_t* buf,
                                                        uint16_t length,
                                                        uint32_t rxAfterTxTimeUs,
                                                        uint16_t rxTimeoutUs) {
    deca_selectDevice(m_spiDevice);
    dwt_setrxaftertxdelay(rxAfterTxTimeUs);
    dwt_setrxtimeout(rxTimeoutUs);

    if (!transmitInternal(buf, length, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED)) {
        return {};
    }

    return receiveInternal(0, 0, true);
}

std::optional<UWBRxStatus> Decawave::transmitDelayedAndReceive(uint8_t* buf,
                                                               uint16_t length,
                                                               uint64_t txTimestamp,
                                                               uint32_t rxAfterTxTimeUs,
                                                               uint16_t rxTimeoutUs) {
    deca_selectDevice(m_spiDevice);
    dwt_setrxaftertxdelay(rxAfterTxTimeUs);
    dwt_setdelayedtrxtime(txTimestamp >> 8);
    dwt_setrxtimeout(rxTimeoutUs);

    if (!transmitInternal(buf, length, DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED)) {
        return {};
    }

    return receiveInternal(0, 0, true);
}

std::optional<UWBRxStatus> Decawave::transmitAndReceiveDelayed(uint8_t* buf,
                                                               uint16_t length,
                                                               uint32_t rxStartDelayUS,
                                                               uint16_t rxTimeoutUs) {
    deca_selectDevice(m_spiDevice);
    dwt_setrxaftertxdelay(rxStartDelayUS);
    dwt_setrxtimeout(rxTimeoutUs);

    if (!transmitInternal(buf, length, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED)) {
        return {};
    }

    return receiveInternal(0, 0, true);
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

void Decawave::getRxTimestamp(uint64_t* rxTimestamp) const {
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

bool Decawave::isReady() const { return m_isReady; }

void Decawave::retrieveRxFrame(UWBRxFrame& frame) const {
    deca_selectDevice(m_spiDevice);

    frame.m_length = m_callbackData.datalength;
    constexpr uint8_t registerDataSize = 26;
    uint8_t registerData[registerDataSize];

    // 0x10
    frame.m_rxPreambleCount =
        (dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT;

    // Read the frame into memory without the CRC16 located at the end of the frame
    // 0x11
    dwt_readrxdata(frame.m_rxBuffer.data(), m_callbackData.datalength - UWB_CRC_LENGTH, 0);

    /*
     * Read all necessary registers in one shot
     * 0:7 -> Register 0x12 Rx Frame Quality Information
     * 8:11 -> Register 0x13 Receiver Time Tracking Interval
     * 12:16 -> Register 0x14 Receive Time Tracking Offset
     * 17:25 -> Register 0x15 Receive Time Stamp
     */
    dwt_readfromdevice(0x12, 0, registerDataSize, registerData);

    // 0x12
    frame.m_stdNoise = ((uint16_t)registerData[1] << 8) + registerData[0];
    frame.m_fpAmpl2 = ((uint16_t)registerData[3] << 8) + registerData[2];
    frame.m_fpAmpl3 = ((uint16_t)registerData[5] << 8) + registerData[4];
    frame.m_cirPwr = ((uint16_t)registerData[7] << 8) + registerData[6];

    // 0x14
    frame.m_sfdAngleRegister = registerData[16];

    // 0x15
    memcpy(&frame.m_rxTimestamp, &(registerData[17]), sizeof(uint64_t));
    frame.m_rxTimestamp &= 0x000000FFFFFFFFFF; // Mask of garbage data at start of TS

    uint16_t firstPathIdx = ((uint16_t)registerData[23] << 8) + registerData[22];
    frame.m_fpAmpl1 = ((uint16_t)registerData[25] << 8) + registerData[24];

    // Compute first path register index and read from accumulator
    firstPathIdx = ((int)(((float)firstPathIdx) * (1.0F / 64.0F) + 0.5F)) * 4.0F;
    // Read one extra byte as readaccdata() returns a dummy byte
    dwt_readfromdevice(ACC_MEM_ID, firstPathIdx, 5, frame.m_firstPathAccumulator);
}

UWBRxStatus Decawave::getRxStatus() const { return m_rxStatus; }

UWBRxStatus Decawave::awaitRx() {
    m_trxTaskHandle = xTaskGetCurrentTaskHandle();

    if (m_rxStatus == UWBRxStatus::ONGOING) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    return m_rxStatus;
}

void Decawave::abortTRXFromISR() {
    deca_selectDevice(m_spiDevice);
    dwt_forcetrxoff();

    m_rxStatus = UWBRxStatus::ERROR;

    BaseType_t taskWoken = pdFALSE;

    if (m_trxTaskHandle != nullptr) {
        vTaskNotifyGiveFromISR(m_trxTaskHandle, &taskWoken);
    }

    m_trxTaskHandle = nullptr;
    portYIELD_FROM_ISR(taskWoken);
}
