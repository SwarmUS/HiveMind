#include "HostUart.h"
#include "hal/uart_host.h"
#include <LockGuard.h>
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
    m_crc(crc),
    m_logger(logger),
    m_hostUartTask("host_uart_process", tskIDLE_PRIORITY + 1, HostUart_processTask, this),
    m_txState(TxState::Idle),
    m_streamMutex(10),
    m_rxState(RxState::Idle),
    m_uartMutex(10) {

    CircularBuff_init(&m_stream, m_streamMemory.data(), m_streamMemory.size());

    startHeaderListen();

    m_hostUartTask.start();
}

bool HostUart::send(const uint8_t* buffer, uint16_t length) {
    if (m_txState != TxState::Idle) {
        m_logger.log(LogLevel::Warn, "Host UART is already busy. Aborting send.");
        return false;
    }

    if (buffer == nullptr || length > m_txBuffer.size()) {
        m_logger.log(LogLevel::Warn, "Invalid parameters for UART send");
        return false;
    }

    m_txLength = length;
    memcpy(m_txBuffer.data(), buffer, length);

    uint32_t crc = m_crc.calculateCRC32(buffer, length);
    *(uint16_t*)m_txHeader.data() = length;
    *(uint32_t*)(m_txHeader.data() + sizeof(length)) = crc;

    LockGuard lock(m_uartMutex);
    bool ret = UartHost_transmitBuffer(m_txHeader.data(), m_txHeader.size(),
                                       (uartCallbackFct)(&hostUart_C_txCpltCallback), (void*)this);

    if (ret) {
        m_txState = TxState::SendHeader;
    } else {
        m_logger.log(LogLevel::Warn, "Could not send message to host over UART");
    }

    return ret;
}

bool HostUart::receive(uint8_t* buffer, uint16_t length) {
    if (buffer == nullptr || length > m_streamMemory.size()) {
        m_logger.log(LogLevel::Warn, "Invalid parameters for UART receive");
        return false;
    }

    m_receivingTaskHandle = xTaskGetCurrentTaskHandle();
    while (CircularBuff_getLength(&m_stream) < length) {
        // Gets notified everytime a new packet is appended to stream
        ulTaskNotifyTake(pdTRUE, 500);
    }
    m_receivingTaskHandle = NULL;

    LockGuard lock = LockGuard(m_streamMutex);
    CircularBuff_get(&m_stream, buffer, length);
    return true;
}

void HostUart::startHeaderListen() {

    if (m_uartMutex.lock()) {
        UartHost_receiveDMA(m_rxHeader.data(), m_rxHeader.size(),
                            (uartCallbackFct)(&hostUart_C_rxCpltCallback), (void*)this);

        m_rxState = RxState::WaitForHeader;

        m_uartMutex.unlock();
    } else {
        m_rxState = RxState::Idle;
    }
}

void HostUart::txCpltCallback() {
    switch (m_txState) {
    case TxState::SendHeader:
        UartHost_transmitBuffer(m_txBuffer.data(), m_txLength,
                                (uartCallbackFct)(&hostUart_C_txCpltCallback), (void*)this);
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
        m_rxLength = *(uint16_t*)m_rxHeader.data();
        m_rxCrc = *(uint32_t*)(m_rxHeader.data() + sizeof(m_rxLength));
        m_rxState = RxState::WaitForPayload;
        UartHost_receiveDMA(m_rxBuffer.data(), m_rxLength,
                            (uartCallbackFct)(&hostUart_C_rxCpltCallback), (void*)this);
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
        uint32_t calculatedCrc = m_crc.calculateCRC32(m_rxBuffer.data(), m_rxLength);
        if (calculatedCrc == m_rxCrc) {
            if (CircularBuff_getFreeSize(&m_stream) >= m_rxLength) {
                // TODO: Send ACK
                addPacketToStream();
            } else {
                // TODO: Send NACK (buffer full)
                m_logger.log(LogLevel::Warn, "UART stream full. Discarding received packet.");
            }

        } else {
            // TODO: Send NACK (invalid CRC)
            m_logger.log(LogLevel::Warn, "Received UART message with incorrect CRC. Discarding.");
        }

        startHeaderListen();
    } else if (m_rxState == RxState::Idle) {
        startHeaderListen();
    }
}

void HostUart::addPacketToStream() {
    LockGuard lock = LockGuard(m_streamMutex);
    CircularBuff_put(&m_stream, m_rxBuffer.data(), m_rxLength);

    // Notify task that is waiting on stream that a new packet is available
    if (m_receivingTaskHandle != NULL) {
        xTaskNotifyGive(m_receivingTaskHandle);
    }
}

bool HostUart::isBusy() const { return m_txState != TxState::Idle; }
