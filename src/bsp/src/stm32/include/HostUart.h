#ifndef __HOSTUART_H__
#define __HOSTUART_H__

#include "bsp/ICRC.h"
#include "bsp/IHostUart.h"
#include <BaseTask.h>
#include <FreeRTOS.h>
#include <Mutex.h>
#include <array>
#include <c-common/circular_buff.h>
#include <cstdint>
#include <logger/ILogger.h>

class HostUart : public IHostUart {
  public:
    explicit HostUart(ICRC& crc, ILogger& logger);
    ~HostUart() override = default;

    bool send(const uint8_t* buffer, uint16_t length) override;
    bool receive(uint8_t* buffer, uint16_t length) override;
    bool isBusy() const override;

    void process();

    friend void hostUart_C_txCpltCallback(void* hostUartInstance);
    friend void hostUart_C_rxCpltCallback(void* hostUartInstance);

  private:
    enum class RxState { Idle, WaitForHeader, WaitForPayload, CheckIntegrity };
    enum class TxState { Idle, SendHeader, SendPayload };

    ICRC& m_crc;
    ILogger& m_logger;

    BaseTask<configMINIMAL_STACK_SIZE * 3> m_hostUartTask;

    TxState m_txState;
    uint8_t m_txLength;
    std::array<uint8_t, HOST_UART_HEADER_LENGTH> m_txHeader;
    std::array<uint8_t, HOST_UART_HEADER_LENGTH> m_rxHeader;

    std::array<uint8_t, HOST_UART_MAX_MESSAGE_LENGTH> m_txBuffer;
    std::array<uint8_t, HOST_UART_MAX_MESSAGE_LENGTH> m_rxBuffer;

    Mutex m_streamMutex;
    std::array<uint8_t, HOST_UART_STREAM_SIZE> m_streamMemory;
    TaskHandle_t m_receivingTaskHandle;
    CircularBuff m_stream;

    uint16_t m_rxLength;
    uint32_t m_rxCrc;
    RxState m_rxState;

    Mutex m_uartMutex;

    void startHeaderListen();
    void txCpltCallback();
    void rxCpltCallback();

    void addPacketToStream();
};

#endif //__HOSTUART_H__
