#include "HostUart.h"
#include "hal/uart_host.h"
#include <FreeRTOSConfig.h>
#include <cstdio>
#include <cstring>
#include <task.h>

void hostUart_C_txCpltCallback(void* hostUartInstance) {
    static_cast<HostUart*>(hostUartInstance)->txCpltCallback();
}

void hostUart_C_rxCpltCallback(void* hostUartInstance) {
    static_cast<HostUart*>(hostUartInstance)->rxCpltCallback();
}

void HostUart_processTask(void* param) {
    const int loopRate = 5;

    while (true) {
        static_cast<HostUart*>(param)->process();
        vTaskDelay(loopRate);
    }
}

HostUart::HostUart(ICRC& crc, ILogger& logger) :
    m_crc(crc), m_logger(logger), m_txState(TxState::Tx_Idle), m_rxState(RxState::Rx_Idle) {
    // TODO: Make static
    m_uartSemaphore = xSemaphoreCreateBinary();

    if (m_uartSemaphore == NULL) {
        m_logger.log(LogLevel::Error, "Host UART semaphore could not be created");
    }

    xSemaphoreGive(m_uartSemaphore);
    startHeaderListen();

    // TODO: Define priorities in a central place (probably in the main cleanup task)
    xTaskCreate(HostUart_processTask, "host_uart_process", configMINIMAL_STACK_SIZE * 3,
                (void*)this, tskIDLE_PRIORITY + 1, NULL);
}

bool HostUart::send(const uint8_t* buffer, uint16_t length) {
    bool ret = false;

    if (m_txState != TxState::Tx_Idle) {
        m_logger.log(LogLevel::Warn, "Host UART is already busy. Aborting send.");
        return false;
    }

    m_txLength = length;
    m_txBuffer = buffer;

    uint32_t crc = m_crc.calculateCRC32(buffer, length);
    *(uint16_t*)m_txHeader = length;
    *(uint32_t*)(m_txHeader + sizeof(length)) = crc;

    if (xSemaphoreTake(m_uartSemaphore, (TickType_t)10) == pdTRUE) {
        ret = UartHost_transmitBuffer(m_txHeader, HOST_UART_HEADER_LENGTH,
                                      (void (*)(void*))(&hostUart_C_txCpltCallback), (void*)this);

        xSemaphoreGive(m_uartSemaphore);
    }

    if (ret) {
        m_txState = TxState::Tx_SendHeader;
    } else {
        m_logger.log(LogLevel::Warn, "Could not send message to host over UART");
    }

    return ret;
}

void HostUart::startHeaderListen() {
    if (xSemaphoreTake(m_uartSemaphore, (TickType_t)10) == pdTRUE) {
        UartHost_receiveDMA(m_rxHeader, HOST_UART_HEADER_LENGTH,
                            (void (*)(void*))(&hostUart_C_rxCpltCallback), (void*)this);

        m_rxState = RxState::Rx_WaitForHeader;

        xSemaphoreGive(m_uartSemaphore);
    } else {
        m_rxState = RxState::Rx_Idle;
    }
}

void HostUart::txCpltCallback() {
    switch (m_txState) {
    case TxState::Tx_SendHeader:
        UartHost_transmitBuffer(m_txBuffer, m_txLength,
                                (void (*)(void*))(&hostUart_C_txCpltCallback), (void*)this);
        m_txState = TxState::Tx_SendPayload;
        break;

    case TxState::Tx_SendPayload:
        m_txState = TxState::Tx_Idle;
        break;

        // Should never get to this state. If it does, reset the driver to send a new message.
    default:
        m_txState = TxState::Tx_Idle;
        break;
    }
}

void HostUart::rxCpltCallback() {
    switch (m_rxState) {
    case RxState::Rx_WaitForHeader:
        m_rxLength = *(uint16_t*)m_rxHeader;
        m_rxCrc = *(uint32_t*)(m_rxHeader + sizeof(m_rxLength));
        m_rxState = RxState::Rx_WaitForPayload;
        UartHost_receiveDMA(m_rxBuffer, m_rxLength, (void (*)(void*))(&hostUart_C_rxCpltCallback),
                            (void*)this);
        break;

    case RxState::Rx_WaitForPayload:
        m_rxState = RxState::Rx_CheckIntegrity;
        break;
    default:
        break;
    }
}

void HostUart::process() {
    if (m_rxState == RxState::Rx_CheckIntegrity) {
        uint32_t calculatedCrc = m_crc.calculateCRC32(m_rxBuffer, m_rxLength);
        if (calculatedCrc == m_rxCrc) {
            m_logger.log(LogLevel::Info, "Received UART message of %d bytes", m_rxLength);
        } else {
            m_logger.log(LogLevel::Warn, "Received UART message with incorrect CRC. Discarding.");
        }

        startHeaderListen();
    } else if (m_rxState == RxState::Rx_Idle) {
        startHeaderListen();
    }
}

bool HostUart::isBusy() { return m_txState != TxState::Tx_Idle; }
