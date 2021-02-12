#include "SpiEsp.h"
#include "hal/esp_spi.h"
#include "hal/hal_gpio.h"
#include <FreeRTOSConfig.h>
#include <cstring>

bool g_testSemaphoreLocked = false;

void task(void* instance) {
    const int loopRate = 30;
    while (true) {
        static_cast<SpiEsp*>(instance)->execute();
        vTaskDelay(loopRate);
    }
}

SpiEsp::SpiEsp(ICRC& crc, ILogger& logger) : m_crc(crc), m_logger(logger) {
    m_txState = transmitState::IDLE;
    m_rxState = receiveState::IDLE;
    m_inboundMessage = {};
    m_outboundMessage = {};
    m_inboundRequest = false;
    m_isBusy = false;

    m_semaphore = xSemaphoreCreateBinaryStatic(&m_semaphoreBuffer);
    xSemaphoreGive(m_semaphore);
    g_testSemaphoreLocked = false;

    setEspCallback(SpiEsp::espInterruptCallback, this);

    m_taskHandle = xTaskCreateStatic(task, "esp_spi_driver_task", 1024u, this, 1u,
                                     (StackType_t*)m_stackData.data(), &m_stackBuffer);
}

bool SpiEsp::send(const uint8_t* buffer, uint16_t length) {
    bool retVal = false;

    if (isBusy()) { // Not available
    } else if (length >= ESP_SPI_MAX_MESSAGE_LENGTH) { // Message too long
        m_logger.log(LogLevel::Warn,
                     "SpiEsp: Message length of %d is larger than maximum allowed of %d", length,
                     ESP_SPI_MAX_MESSAGE_LENGTH);
    } else {

        // TODO: this way of storing a pointer the buffer will be fixed once we have a buffer
        m_logger.log(LogLevel::Debug, "Sending message of length %d", length);
        // Memcpy necessary to have buffer word-alligned for transfer
        std::memcpy(m_outboundMessage.m_data.data(), buffer, length);
        // Padding with 0 up to a word-alligned boundary
        for (uint8_t i = 0; i < (length % 4); i++) {
            m_outboundMessage.m_data[length] = 0;
            length++;
        }
        // Appending CRC32
        *(uint32_t*)(m_outboundMessage.m_data.data() + length) =
            m_crc.calculateCRC32(buffer, length);
        m_outboundMessage.m_sizeBytes = length;
        m_txState = transmitState::SENDING_HEADER;
        m_isBusy = true;
        retVal = true;
        if (eTaskGetState(m_taskHandle) == eSuspended) {
            vTaskResume(m_taskHandle);
        }
    }

    return retVal;
}

bool SpiEsp::isBusy() const { return m_isBusy; }

void SpiEsp::execute() {
    uint32_t txLengthBytes = 0;
    uint32_t rxLengthBytes = 0;
    auto* txBuffer = (uint8_t*)&m_outboundHeader; // Send header by default;

    switch (m_rxState) {
    case receiveState::IDLE:
        if (m_inboundRequest || m_txState != transmitState::IDLE) {
            rxLengthBytes = 4;
            m_rxState = receiveState::RECEIVING_HEADER;
        }
        break;
    case receiveState::RECEIVING_HEADER:
        rxLengthBytes = EspSpi::headerSize;
        break;
    case receiveState::PARSING_HEADER:
        m_inboundHeader = (EspSpi::Header*)m_inboundMessage.m_data.data();
        if (m_inboundHeader->headerStruct.crc8 != m_crc.calculateCRC8(m_inboundHeader, 3)) {
            m_logger.log(LogLevel::Error, "Received corrupted header");
            m_logger.log(LogLevel::Debug, "Bytes were: | %d | %d | %d | %d |",
                         m_inboundMessage.m_data[0], m_inboundMessage.m_data[1],
                         m_inboundMessage.m_data[2], m_inboundMessage.m_data[3]);
            m_rxState = receiveState::ERROR;
            break;
        }
        if (m_inboundHeader->headerStruct.rxSizeWord << 2 == m_outboundMessage.m_sizeBytes &&
            m_outboundMessage.m_sizeBytes != 0) {
            m_logger.log(LogLevel::Debug, "Received valid header. Can now send payload");
            m_txState = transmitState::SENDING_PAYLOAD;
        } else {
            m_txState = transmitState::SENDING_HEADER;
            m_logger.log(LogLevel::Debug, "Received valid header but cannot send payload");
        }
        // This will be sent on next header. Payload has priority over headers.
        m_inboundMessage.m_sizeBytes = m_inboundHeader->headerStruct.txSizeWord << 2;
        if (m_inboundMessage.m_sizeBytes == m_outboundHeader.headerStruct.rxSizeWord << 2 &&
            m_inboundMessage.m_sizeBytes != 0) {
            rxLengthBytes = m_inboundHeader->headerStruct.txSizeWord << 2;
            m_rxState = receiveState::RECEIVING_PAYLOAD;
        } else {
            rxLengthBytes = 4;
            m_rxState = receiveState::RECEIVING_HEADER;
        }
        break;
    case receiveState::RECEIVING_PAYLOAD:
        rxLengthBytes = m_inboundHeader->headerStruct.txSizeWord << 2;
        break;
    case receiveState::VALIDATE_CRC:
        // TODO: validate CRC
        m_logger.log(LogLevel::Info, "ESP says: %s", m_inboundMessage.m_data.data());
        m_inboundMessage.m_sizeBytes = 0;

        m_rxState = receiveState::RECEIVING_HEADER;
        rxLengthBytes = EspSpi::headerSize;
        break;
    case receiveState::ERROR:
        rxLengthBytes = 4;
        m_rxState = receiveState::RECEIVING_HEADER;
        break;
    }
    if (m_inboundRequest && m_txState == transmitState::IDLE) {
        m_txState = transmitState::SENDING_HEADER;
    }

    // Transmitting state machine
    switch (m_txState) {
    case transmitState::IDLE:
        break;
    case transmitState::SENDING_HEADER:
        updateOutboundHeader();
        txLengthBytes = EspSpi::headerSize;
        txBuffer = (uint8_t*)&m_outboundHeader;
        break;
    case transmitState::SENDING_PAYLOAD:
        txLengthBytes = m_outboundMessage.m_sizeBytes;
        txBuffer = m_outboundMessage.m_data.data();
        break;
    case transmitState::ERROR:
        HAL_GPIO_WritePin(ESP_CS_GPIO_Port, ESP_CS_Pin, GPIO_PIN_SET);
        break;
    }

    if ((m_inboundRequest || m_outboundMessage.m_sizeBytes != 0) &&
        m_txState != transmitState::ERROR && m_rxState != receiveState::ERROR) {
        HAL_GPIO_WritePin(ESP_CS_GPIO_Port, ESP_CS_Pin, GPIO_PIN_RESET);
        vTaskDelay(20);
        uint32_t finalSize = std::max(txLengthBytes, rxLengthBytes);
        memset(m_inboundMessage.m_data.data(), 0, ESP_SPI_MAX_MESSAGE_LENGTH);
        if (xSemaphoreTake(m_semaphore, (TickType_t)10) == pdTRUE) {
            EspSpi_TransmitReceiveDma(txBuffer, m_inboundMessage.m_data.data(), finalSize,
                                      SpiEsp::espTxRxCallback, this);
            g_testSemaphoreLocked = false;
            xSemaphoreGive(m_semaphore);
        }
    }
}

void SpiEsp::updateOutboundHeader() {
    // TODO: get actual system state
    m_outboundHeader.headerStruct.systemState.rawValue = 128u;
    m_outboundHeader.headerStruct.systemState.stmSystemState.failedCrc = false;
    m_outboundHeader.headerStruct.rxSizeWord = m_inboundMessage.m_sizeBytes >> 2;
    m_outboundHeader.headerStruct.txSizeWord = m_outboundMessage.m_sizeBytes >> 2;
    m_outboundHeader.headerStruct.crc8 = m_crc.calculateCRC8(&m_outboundHeader, 3);
    if (m_outboundHeader.headerStruct.txSizeWord == 0) {
        m_isBusy = false;
    }
}

void SpiEsp::espInterruptCallback(void* instance) {
    auto* t = static_cast<SpiEsp*>(instance);
    // ESP has stuff to say
    t->m_inboundRequest = HAL_GPIO_ReadPin(ESP_CS_GPIO_Port, ESP_CS_Pin);
    /*if (eTaskGetState(t->m_taskHandle) == eSuspended) {
        portYIELD_FROM_ISR(xTaskResumeFromISR(t->m_taskHandle))
    }*/
}

void SpiEsp::espTxCallback(void* instance) {
    auto* _this = static_cast<SpiEsp*>(instance);
    if (_this->m_txState == transmitState::SENDING_PAYLOAD) { // TODO: confirm reception with ack
        _this->m_txState = transmitState::SENDING_HEADER;
    }
}

void SpiEsp::espRxCallback(void* instance) {
    auto* _this = static_cast<SpiEsp*>(instance);
    switch (_this->m_rxState) {
    case receiveState::RECEIVING_HEADER:
        _this->m_rxState = receiveState::PARSING_HEADER;
        break;
    case receiveState::RECEIVING_PAYLOAD:
        _this->m_rxState = receiveState::VALIDATE_CRC;
        break;
    default:
        // other states should not be present on callback
        break;
    }
}

void SpiEsp::espTxRxCallback(void* instance) {
    auto* _this = static_cast<SpiEsp*>(instance);
    // BaseType_t xHigherPriorityTaskWoken;

    HAL_GPIO_WritePin(ESP_CS_GPIO_Port, ESP_CS_Pin, GPIO_PIN_SET);
    switch (_this->m_rxState) {
    case receiveState::RECEIVING_HEADER:
        _this->m_rxState = receiveState::PARSING_HEADER;
        break;
    case receiveState::RECEIVING_PAYLOAD:
        _this->m_rxState = receiveState::VALIDATE_CRC;
        break;
    default:
        // other states should not be present on callback
        break;
    }

    if (_this->m_txState == transmitState::SENDING_PAYLOAD) { // TODO: confirm reception with ack
        _this->m_txState = transmitState::IDLE;
        _this->m_outboundMessage.m_sizeBytes = 0;
    }
    // The higherPriorityTaskWoken is imply a boolean passed by reference used.
    // It is used to determine if a yield needs to occur for context switch after this ISR.
    /* xSemaphoreGiveFromISR(_this->m_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    g_testSemaphoreLocked = false; */
}