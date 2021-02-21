#include "SpiEsp.h"
#include "hal/esp_spi.h"
#include "hal/hal_gpio.h"
#include <FreeRTOSConfig.h>
#include <cstring>

#define WORD_TO_BYTE(word) ((uint32_t) (word << 2U))
#define BYTE_TO_WORD(byte) ((uint8_t) (byte >> 2U))

void task(void* instance) {
    constexpr uint loopRate = 30;
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

    setEspCallback(SpiEsp::espInterruptCallback, this);

    m_taskHandle = xTaskCreateStatic(task, "esp_spi_driver_task", 1024U, this, 1U,
                                     (StackType_t*)m_stackData.data(), &m_stackBuffer);
}

bool SpiEsp::send(const uint8_t* buffer, uint16_t length) {
    if (isBusy()) { // Not available
    } else if (length >= ESP_SPI_MAX_MESSAGE_LENGTH) { // Message too long
        m_logger.log(LogLevel::Warn,
                     "SpiEsp: Message length of %d is larger than maximum allowed of %d", length,
                     ESP_SPI_MAX_MESSAGE_LENGTH);
    } else {

        m_logger.log(LogLevel::Debug, "Sending message of length %d", length);
        // TODO: this memcpy could change once we have a buffer manager/allocator of some sorts.
        std::memcpy(m_outboundMessage.m_data.data(), buffer, length);
        // Padding with 0 up to a word-alligned boundary
        for (uint8_t i = 0; i < (length % 4); i++) {
            m_outboundMessage.m_data[length] = 0;
            length++;
        }
        // Appending CRC32
        *(uint32_t*)(m_outboundMessage.m_data.data() + length) =
            m_crc.calculateCRC32(m_outboundMessage.m_data.data(), length);
        length += CRC32_SIZE;
        m_outboundMessage.m_sizeBytes = length;
        m_txState = transmitState::SENDING_HEADER;
        m_isBusy = true;
        return true;
    }

    return false;
}

bool SpiEsp::isBusy() const { return m_isBusy; }

void SpiEsp::execute() {
    uint32_t txLengthBytes = 0;
    uint32_t rxLengthBytes = 0;
    auto* txBuffer = (uint8_t*)&m_outboundHeader; // Send header by default;

    switch (m_rxState) {
    case receiveState::IDLE:
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
            m_logger.log(LogLevel::Error, "Received corrupted SPI ESP header");
            m_logger.log(LogLevel::Debug, "Bytes were: | %d | %d | %d | %d |",
                         m_inboundMessage.m_data[0], m_inboundMessage.m_data[1],
                         m_inboundMessage.m_data[2], m_inboundMessage.m_data[3]);
            m_rxState = receiveState::ERROR;
            break;
        }
        if (WORD_TO_BYTE(m_inboundHeader->rxSizeWord) == m_outboundMessage.m_sizeBytes &&
            m_outboundMessage.m_sizeBytes != 0) {
            m_logger.log(LogLevel::Debug, "Received valid header. Can now send payload");
            m_txState = transmitState::SENDING_PAYLOAD;
        } else {
            m_txState = transmitState::SENDING_HEADER;
            m_logger.log(LogLevel::Debug, "Received valid header but cannot send payload");
        }
        // This will be sent on next header. Payload has priority over headers.
        m_inboundMessage.m_sizeBytes = WORD_TO_BYTE(m_inboundHeader->txSizeWord);
        if (m_inboundMessage.m_sizeBytes ==  WORD_TO_BYTE(m_outboundHeader.rxSizeWord) &&
            m_inboundMessage.m_sizeBytes != 0) {
            rxLengthBytes = WORD_TO_BYTE(m_inboundHeader->txSizeWord);
            m_outboundHeader.systemState.stmSystemState.failedCrc = 0;
            m_rxState = receiveState::RECEIVING_PAYLOAD;
        } else {
            rxLengthBytes = EspHeader::sizeBytes;
            m_rxState = receiveState::RECEIVING_HEADER;
        }
        break;
    case receiveState::RECEIVING_PAYLOAD:
        rxLengthBytes = WORD_TO_BYTE(m_inboundHeader->txSizeWord);
        break;
    case receiveState::VALIDATE_CRC:
        if (m_crc.calculateCRC32(m_inboundMessage.m_data.data(),
                                 m_inboundMessage.m_sizeBytes - CRC32_SIZE) !=
            *(uint32_t*)&m_inboundMessage.m_data[m_inboundMessage.m_sizeBytes - CRC32_SIZE]) {
            m_outboundHeader.systemState.stmSystemState.failedCrc = 1;
        } else {
            m_logger.log(LogLevel::Info, "ESP says: %s", m_inboundMessage.m_data.data());
            m_inboundMessage.m_sizeBytes = 0;
        }
        m_rxState = receiveState::RECEIVING_HEADER;
        rxLengthBytes = EspHeader::sizeBytes;
        break;
    case receiveState::ERROR:
        rxLengthBytes = EspHeader::sizeBytes;
        m_rxState = receiveState::RECEIVING_HEADER;
        break;
    }
    if (m_inboundRequest && m_txState == transmitState::IDLE) {
        m_txState = transmitState::SENDING_HEADER;
    }

    // Transmitting state machine
    switch (m_txState) {
    case transmitState::IDLE:
        m_isBusy = false;
        break;
    case transmitState::SENDING_HEADER:
        updateOutboundHeader();
        txLengthBytes = EspHeader::sizeBytes;
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
        uint32_t finalSize = std::max(txLengthBytes, rxLengthBytes);
        memset(m_inboundMessage.m_data.data(), 0, ESP_SPI_MAX_MESSAGE_LENGTH);
        EspSpi_TransmitReceiveDma(txBuffer, m_inboundMessage.m_data.data(), finalSize,
                                  SpiEsp::espTxRxCallback, this);
    }
}

void SpiEsp::updateOutboundHeader() {
    // TODO: get actual system state
    m_outboundHeader.rxSizeWord = m_inboundMessage.m_sizeBytes >> 2U;
    m_outboundHeader.txSizeWord = m_outboundMessage.m_sizeBytes >> 2U;
    m_outboundHeader.crc8 = m_crc.calculateCRC8(&m_outboundHeader, EspHeader::sizeBytes - 1);
    if (m_outboundHeader.txSizeWord == 0) {
        m_isBusy = false;
    }
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
        // other states will not be present on this callback
        break;
    }

    if (instance->m_txState == transmitState::SENDING_PAYLOAD) { // TODO: confirm reception with ack
        instance->m_txState = transmitState::IDLE;
        instance->m_outboundMessage.m_sizeBytes = 0;
    }
}