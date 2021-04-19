#include "SpiEsp.h"
#include "hal/esp_spi.h"
#include "hal/hal_gpio.h"
#include <cstring>

void task(void* context) {
    constexpr uint16_t loopRate = 20;
    while (true) {
        static_cast<SpiEsp*>(context)->execute();
        Task::delay(loopRate);
    }
}

SpiEsp::SpiEsp(ICRC& crc, ILogger& logger) :
    m_driverTask("esp_spi_driver", tskIDLE_PRIORITY + 1, task, this), m_crc(crc), m_logger(logger) {

    m_txState = transmitState::IDLE;
    m_rxState = receiveState::IDLE;
    m_inboundMessage = {};
    m_outboundMessage = {};
    m_inboundRequest = false;
    m_isConnected = false;
    m_crcOK = false;
    CircularBuff_init(&m_circularBuf, m_data.data(), m_data.size());
    setEspCallback(SpiEsp::espInterruptCallback, this);

    // Starts the task
    m_driverTask.start();
}

bool SpiEsp::send(const uint8_t* buffer, uint16_t length) {
    if (length >= ESP_SPI_MAX_MESSAGE_LENGTH) { // Message too long
        m_logger.log(LogLevel::Warn,
                     "SpiEsp: Message length of %d is larger than maximum allowed of %d", length,
                     ESP_SPI_MAX_MESSAGE_LENGTH);
        return false;
    }

    m_logger.log(LogLevel::Debug, "Sending message of length %d", length);
    // TODO: this memcpy could change once we have a buffer manager/allocator of some sorts.
    std::memcpy(m_outboundMessage.m_data.data(), buffer, length);
    // Set payload size
    m_outboundMessage.m_payloadSize = length;
    // Padding with 0 up to a word-alligned boundary
    while (length % 4 != 0) {
        m_outboundMessage.m_data[(uint16_t)(length)] = 0;
        length++;
    }
    // Appending CRC32
    *(uint32_t*)&m_outboundMessage.m_data[length] =
        m_crc.calculateCRC32(m_outboundMessage.m_data.data(), length);
    m_outboundMessage.m_sizeBytes = (uint16_t)(length + CRC32_SIZE);
    m_txState = transmitState::SENDING_HEADER;

    // Wait for transmission to be over
    m_sendingTaskHandle = xTaskGetCurrentTaskHandle();
    // Wait for transmission to be over. Will be notified when ACK received or upon error
    m_sendingTaskHandle = xTaskGetCurrentTaskHandle();
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (m_txState == transmitState::ERROR) {
        m_logger.log(LogLevel::Error, "Error occurred...");
        return false;
    }
    m_logger.log(LogLevel::Info, "Payload sent!");
    m_sendingTaskHandle = nullptr;
    return m_crcOK;
}
bool SpiEsp::receive(uint8_t* buffer, uint16_t length) {
    if (buffer == nullptr || length > ESP_SPI_MAX_MESSAGE_LENGTH) {
        m_logger.log(LogLevel::Warn, "Invalid parameters for SpiStm::Receive");
        return false;
    }
    m_receivingTaskHandle = xTaskGetCurrentTaskHandle();
    while (CircularBuff_getLength(&m_circularBuf) < length) {
        ulTaskNotifyTake(pdTRUE, 500);
        // TODO: check for disconnection
    }
    m_receivingTaskHandle = nullptr;

    return CircularBuff_get(&m_circularBuf, buffer, length) == length;
}

bool SpiEsp::isConnected() const { return m_isConnected; }

void SpiEsp::execute() {
    uint32_t txLengthBytes = 0;
    uint32_t rxLengthBytes = 0;
    auto* txBuffer = (uint8_t*)&m_outboundHeader; // Send header by default;

    switch (m_rxState) {
    case receiveState::IDLE:
        if (!m_inboundRequest) {
            m_inboundRequest = HAL_GPIO_ReadPin(ESP_CS_GPIO_Port, ESP_CS_Pin) == 1U;
        }
        if (m_inboundRequest || m_txState != transmitState::IDLE) {
            rxLengthBytes = EspHeader::sizeBytes;
            m_rxState = receiveState::RECEIVING_HEADER;
        }
        break;
    case receiveState::RECEIVING_HEADER:
        rxLengthBytes = EspHeader::sizeBytes;
        break;
    case receiveState::PARSING_HEADER:
        m_inboundHeader = (EspHeader::Header*)m_inboundMessage.m_data.data();
        if (m_inboundHeader->crc8 !=
            m_crc.calculateCRC8(m_inboundHeader, EspHeader::sizeBytes - 1)) {
            m_logger.log(LogLevel::Debug, "Received corrupted SPI ESP header");
            m_logger.log(LogLevel::Debug, "Bytes were: | %d | %d | %d | %d |",
                         m_inboundMessage.m_data[0], m_inboundMessage.m_data[1],
                         m_inboundMessage.m_data[2], m_inboundMessage.m_data[3]);
            m_rxState = receiveState::ERROR;
            if (m_hasSentPayload) {
                m_txState = transmitState::ERROR;
            }
            m_isConnected = false;
            break;
        }
        // Simple flag signifying that connection is established with esp
        m_isConnected = true;
        if (m_inboundHeader->rxSizeBytes == m_outboundMessage.m_sizeBytes &&
            m_outboundMessage.m_sizeBytes != 0) {
            m_logger.log(LogLevel::Debug, "Received valid header. Can now send payload");
            m_txState = transmitState::SENDING_PAYLOAD;
        } else {
            m_txState = transmitState::SENDING_HEADER;
            m_logger.log(LogLevel::Debug, "Received valid header but cannot send payload");
        }
        // This will be sent on next header. Payload has priority over headers.
        m_inboundMessage.m_sizeBytes = m_inboundHeader->txSizeBytes;
        if (m_inboundMessage.m_sizeBytes == m_outboundHeader.rxSizeBytes &&
            m_inboundMessage.m_sizeBytes != 0) {
            m_inboundMessage.m_payloadSize = m_inboundHeader->payloadSizeBytes;
            rxLengthBytes = m_inboundHeader->txSizeBytes;
            m_outboundHeader.systemState.stmSystemState.failedCrc = 0;
            m_rxState = receiveState::RECEIVING_PAYLOAD;
        } else {
            rxLengthBytes = EspHeader::sizeBytes;
            m_rxState = receiveState::RECEIVING_HEADER;
        }
        // Payload has been sent. Check crc and notify sending task
        if (m_hasSentPayload) {
            m_hasSentPayload = false;
            m_crcOK = !m_inboundHeader->systemState.stmSystemState.failedCrc;
            if (m_sendingTaskHandle != nullptr) {
                xTaskNotifyGive(m_sendingTaskHandle);
            }
        }
        break;
    case receiveState::RECEIVING_PAYLOAD:
        rxLengthBytes = m_inboundHeader->txSizeBytes;
        break;
    case receiveState::VALIDATE_CRC:
        // Check payload CRC and log an error and set flag if it fails
        if (m_crc.calculateCRC32(m_inboundMessage.m_data.data(),
                                 (uint16_t)(m_inboundMessage.m_sizeBytes - CRC32_SIZE)) !=
            *(uint32_t*)&m_inboundMessage
                 .m_data[(uint16_t)(m_inboundMessage.m_sizeBytes - CRC32_SIZE)]) {
            m_logger.log(LogLevel::Error, "Failed payload crc on ESP");
            m_outboundHeader.systemState.stmSystemState.failedCrc = 1;
        }
        // If it passes the CRC check, add the data to the circular buffer
        else if (CircularBuff_put(&m_circularBuf, m_inboundMessage.m_data.data(),
                                  m_inboundMessage.m_payloadSize) == CircularBuff_Ret_Ok) {
            // If a task was waiting to receive bytes, notify it
            if (m_receivingTaskHandle != nullptr) {
                xTaskNotifyGive(m_receivingTaskHandle);
            }
        } else {
            m_logger.log(LogLevel::Error, "Failed to add bytes in spi circular buffer");
        }
        m_inboundMessage.m_payloadSize = 0;
        m_inboundMessage.m_sizeBytes = 0;
        m_rxState = receiveState::RECEIVING_HEADER;
        rxLengthBytes = EspHeader::sizeBytes;
        break;
    case receiveState::ERROR:
        rxLengthBytes = EspHeader::sizeBytes;
        m_rxState = receiveState::RECEIVING_HEADER;
        CircularBuff_clear(&m_circularBuf);
        if (m_receivingTaskHandle != nullptr) {
            xTaskNotifyGive(m_receivingTaskHandle);
        }
        m_logger.log(LogLevel::Error, "Error within Spi driver ESP - RX");
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
        txLengthBytes = EspHeader::sizeBytes;
        txBuffer = (uint8_t*)&m_outboundHeader;
        break;
    case transmitState::SENDING_PAYLOAD:
        txLengthBytes = m_outboundMessage.m_sizeBytes;
        txBuffer = m_outboundMessage.m_data.data();
        m_hasSentPayload = false;
        m_crcOK = false;
        break;
    case transmitState::ERROR:
        m_crcOK = false;
        HAL_GPIO_WritePin(ESP_CS_GPIO_Port, ESP_CS_Pin, GPIO_PIN_SET);
        if (m_sendingTaskHandle != nullptr) {
            xTaskNotifyGive(m_sendingTaskHandle);
        }
        m_txState = transmitState::SENDING_HEADER;
        m_logger.log(LogLevel::Error, "Error within Spi driver ESP - TX");
        break;
    }

    if ((m_inboundRequest || m_outboundMessage.m_sizeBytes != 0 ||
         m_inboundMessage.m_sizeBytes != 0) &&
        m_txState != transmitState::ERROR && m_rxState != receiveState::ERROR) {
        HAL_GPIO_WritePin(ESP_CS_GPIO_Port, ESP_CS_Pin, GPIO_PIN_RESET);
        uint32_t finalSize = std::max(txLengthBytes, rxLengthBytes);
        m_inboundMessage.m_data.fill(0);
        EspSpi_TransmitReceiveDma(txBuffer, m_inboundMessage.m_data.data(), finalSize,
                                  SpiEsp::espTxRxCallback, this);
    }
}

void SpiEsp::updateOutboundHeader() {
    // TODO: get actual system state
    m_outboundHeader.rxSizeBytes = m_inboundMessage.m_sizeBytes;
    m_outboundHeader.txSizeBytes = m_outboundMessage.m_sizeBytes;
    m_outboundHeader.payloadSizeBytes = m_outboundMessage.m_payloadSize;
    m_outboundHeader.crc8 = m_crc.calculateCRC8(&m_outboundHeader, EspHeader::sizeBytes - 1);
}

void SpiEsp::espInterruptCallback(void* context) {
    auto* instance = static_cast<SpiEsp*>(context);
    // Interrupt is on both falling edge and rising edge.
    instance->m_inboundRequest = (HAL_GPIO_ReadPin(ESP_CS_GPIO_Port, ESP_CS_Pin) == 1U);
}

void SpiEsp::espTxRxCallback(void* context) {
    auto* instance = static_cast<SpiEsp*>(context);

    HAL_GPIO_WritePin(ESP_CS_GPIO_Port, ESP_CS_Pin, GPIO_PIN_SET);
    switch (instance->m_rxState) {
    case receiveState::RECEIVING_HEADER:
        instance->m_rxState = receiveState::PARSING_HEADER;
        break;
    case receiveState::RECEIVING_PAYLOAD:
        instance->m_rxState = receiveState::VALIDATE_CRC;
        break;
    default:
        // This should never be called. The state machine should never be in any other state during
        // the ISR.
        instance->m_logger.log(LogLevel::Error, "Interrupted called on invalid state");
        instance->m_txState = transmitState::ERROR;
        break;
    }

    if (instance->m_txState == transmitState::SENDING_PAYLOAD) {
        instance->m_txState = transmitState::IDLE;
        instance->m_outboundMessage.m_sizeBytes = 0;
        instance->m_outboundMessage.m_payloadSize = 0;
        instance->m_hasSentPayload = true;
    }
}
