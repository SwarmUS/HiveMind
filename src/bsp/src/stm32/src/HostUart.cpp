#include "HostUart.h"
#include "hal/uart_host.h"
#include <FreeRTOSConfig.h>
#include <cstdio>
#include <cstring>
#include <task.h>

void hostUart_C_txCpltCallback(void* phoneCommunicationInstance) {
    static_cast<HostUart*>(phoneCommunicationInstance)->txCpltCallback();
}

void hostUart_C_rxCpltCallback(void* phoneCommunicationInstance) {
    static_cast<HostUart*>(phoneCommunicationInstance)->rxCpltCallback();
}

void HostUart_processTask(void* param) {
    const int loopRate = 5;

    while (true) {
        static_cast<HostUart*>(param)->process();
        vTaskDelay(loopRate);
    }
}

HostUart::HostUart(ICRC& crc, ILogger& logger) :
    m_crc(crc), m_logger(logger), m_txState(TxState::Idle), m_rxState(RxState::WaitForHeader) {
    m_uartSemaphore = xSemaphoreCreateBinary();

    if (m_uartSemaphore == NULL) {
        m_logger.log(LogLevel::Error, "Host UART semaphore could not be created");
    }

    xSemaphoreGive(m_uartSemaphore);

    // TODO: Define priorities in a central place (probably in the main cleanup task)
    xTaskCreate(HostUart_processTask, "host_uart_process", configMINIMAL_STACK_SIZE * 3,
                (void*)this, tskIDLE_PRIORITY + 1, NULL);

    UartHost_receiveDMA(m_rxHeader, HOST_UART_HEADER_LENGTH,
                        (void (*)(void*))(&hostUart_C_rxCpltCallback), (void*)this);
}

bool HostUart::send(const uint8_t* buffer, uint16_t length) {
    bool ret = false;

    if (m_txState != TxState::Idle) {
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
        m_txState = TxState::SendHeader;
    } else {
        m_logger.log(LogLevel::Warn, "Could not send message to host over UART");
    }

    return ret;
}

void HostUart::txCpltCallback() {
    switch (m_txState) {
    case TxState::SendHeader:
        UartHost_transmitBuffer(m_txBuffer, m_txLength,
                                (void (*)(void*))(&hostUart_C_txCpltCallback), (void*)this);
        m_txState = TxState::SendPayload;
        break;

    case TxState::SendPayload:
        m_txState = TxState::Idle;
        break;

        // Should never get to this state. If it does, reset the driver to send a new message.
    default:
        m_txState = TxState::Idle;
        break;
    }
}

void HostUart::rxCpltCallback() {
    switch (m_rxState) {
    case RxState::WaitForHeader:
        m_rxLength = *(uint16_t*)m_rxHeader;
        m_rxCrc = *(uint32_t*)(m_rxHeader + sizeof(m_rxLength));
        m_rxState = RxState::WaitForPayload;
        UartHost_receiveDMA(m_rxBuffer, m_rxLength, (void (*)(void*))(&hostUart_C_rxCpltCallback),
                            (void*)this);
        break;

    case RxState::WaitForPayload:
        m_rxState = RxState::CheckIntegrity;
        break;
    default:
        break;
    }
}

void HostUart::process() {
    if (m_rxState == RxState::CheckIntegrity) {
        uint32_t calculatedCrc = m_crc.calculateCRC32(m_rxBuffer, m_rxLength);
        if (calculatedCrc == m_rxCrc) {
            m_logger.log(LogLevel::Info, "Received UART message of %d bytes", m_rxLength);
        } else {
            m_logger.log(LogLevel::Warn, "Received UART message with incorrect CRC. Discarding.");
        }

        m_rxState = RxState::WaitForHeader;

        if (xSemaphoreTake(m_uartSemaphore, (TickType_t)10) == pdTRUE) {
            UartHost_receiveDMA(m_rxHeader, HOST_UART_HEADER_LENGTH,
                                (void (*)(void*))(&hostUart_C_rxCpltCallback), (void*)this);

            xSemaphoreGive(m_uartSemaphore);
        }
    }
}

bool HostUart::isBusy() { return m_txState != TxState::Idle; }
