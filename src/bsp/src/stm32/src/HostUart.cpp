#include "HostUart.h"
#include "hal/uart_phone.h"
#include <FreeRTOSConfig.h>
#include <cstdio>
#include <cstring>
#include <task.h>

void phoneCommunication_C_txCpltCallback(void* phoneCommunicationInstance) {
    static_cast<HostUart*>(phoneCommunicationInstance)->txCpltCallback();
}

void phoneCommunication_C_rxCpltCallback(void* phoneCommunicationInstance) {
    static_cast<HostUart*>(phoneCommunicationInstance)->rxCpltCallback();
}

void messageNotifier(void* param) {
    const int loopRate = 5;

    while (true) {
        static_cast<HostUart*>(param)->process();
        vTaskDelay(loopRate);
    }
}

HostUart::HostUart(ICRC& crc, ILogger& logger) :
    m_crc(crc), m_logger(logger), m_busy(false), m_rxState(RxState::waitForHeader) {
    m_uartSemaphore = xSemaphoreCreateBinary();

    if (m_uartSemaphore == NULL) {
        m_logger.log(LogLevel::Error, "Host UART semaphore could not be created");
    }

    xSemaphoreGive(m_uartSemaphore);

    // TODO: Define priorities in a central place
    xTaskCreate(messageNotifier, "phone_communication_process", configMINIMAL_STACK_SIZE,
                (void*)this, tskIDLE_PRIORITY + 1, NULL);

    UartPhone_receiveDMA(m_rxHeader, HOS_UART_HEADER_LENGTH,
                         (void (*)(void*))(&phoneCommunication_C_rxCpltCallback), (void*)this);
}

bool HostUart::sendBytes(const uint8_t* bytes, uint16_t length) {
    bool ret = false;
    if (m_busy) {
        ret = false;
        m_logger.log(LogLevel::Warn, "Host UART is already busy. Aborting send.");
    } else {
        *m_txBuffer = length;
        volatile uint32_t crc = m_crc.calculateCRC32(bytes, length);
        memcpy((m_txBuffer + sizeof(length)), (const void*)&crc, sizeof(crc));
        memcpy((m_txBuffer + sizeof(length) + sizeof(crc)), bytes, length);

        if (xSemaphoreTake(m_uartSemaphore, (TickType_t)10) == pdTRUE) {
            ret = UartPhone_transmitBuffer(m_txBuffer, length + sizeof(length) + sizeof(crc),
                                           (void (*)(void*))(&phoneCommunication_C_txCpltCallback),
                                           (void*)this);

            xSemaphoreGive(m_uartSemaphore);
        }

        if (!ret) {
            m_logger.log(LogLevel::Warn, "Could not send message to host over UART");
        }
    }
    return ret;
}

void HostUart::rxCpltCallback() {
    switch (m_rxState) {
    case RxState::waitForHeader:
        m_rxLength = *(uint16_t*)m_rxHeader;
        m_rxCrc = *(uint32_t*)(m_rxHeader + sizeof(m_rxLength));
        m_rxState = RxState::waitForPayload;
        UartPhone_receiveDMA(m_rxBuffer, m_rxLength,
                             (void (*)(void*))(&phoneCommunication_C_rxCpltCallback), (void*)this);
        break;

    case RxState::waitForPayload:
        m_rxState = RxState::checkIntegrity;
        break;
    default:
        break;
    }
}

void HostUart::process() {
    if (m_rxState == RxState::checkIntegrity) {
        uint32_t calculatedCrc = m_crc.calculateCRC32(m_rxBuffer, m_rxLength);
        if (calculatedCrc == m_rxCrc) {
            // TODO: Callback
        } else {
            m_logger.log(LogLevel::Warn, "Received UART message with incorrect CRC. Discarding.");
        }

        m_rxState = RxState::waitForHeader;

        if (xSemaphoreTake(m_uartSemaphore, (TickType_t)10) == pdTRUE) {
            UartPhone_receiveDMA(m_rxHeader, HOS_UART_HEADER_LENGTH,
                                 (void (*)(void*))(&phoneCommunication_C_rxCpltCallback),
                                 (void*)this);

            xSemaphoreGive(m_uartSemaphore);
        }
    }
}

void HostUart::registerCallback() { return; }
bool HostUart::isBusy() { return m_busy; }

void HostUart::txCpltCallback() { m_busy = false; }
