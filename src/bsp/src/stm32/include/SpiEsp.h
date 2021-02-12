#ifndef __SPIESP_H__
#define __SPIESP_H__

#include "SpiHeader.h"
#include "bsp/ICRC.h"
#include "bsp/ISpiEsp.h"
#include "logger/ILogger.h"
#include <FreeRTOS.h>
#include <array>
#include <semphr.h>

class SpiEsp : public ISpiEsp {
  public:
    SpiEsp(ICRC& crc, ILogger& logger);
    ~SpiEsp() override = default;

    bool send(const uint8_t* buffer, uint16_t length) override;

    bool isBusy() const override;

    void execute();

  protected:
    ICRC& m_crc;
    ILogger& m_logger;

    enum class transmitState { IDLE, SENDING_HEADER, SENDING_PAYLOAD, ERROR } m_txState;
    enum class receiveState {
        IDLE,
        RECEIVING_HEADER,
        PARSING_HEADER,
        RECEIVING_PAYLOAD,
        VALIDATE_CRC,
        ERROR
    } m_rxState;

    struct message {
        std::array<uint8_t, ESP_SPI_MAX_MESSAGE_LENGTH> m_data;
        uint16_t m_sizeBytes;
    } m_inboundMessage, m_outboundMessage;

    EspSpi::Header m_outboundHeader, *m_inboundHeader;

  private:
    static void espInterruptCallback(void* instance);
    static void espTxCallback(void* instance);
    static void espRxCallback(void* instance);
    static void espTxRxCallback(void* instance);

    void updateOutboundHeader();
    bool m_inboundRequest;

    StaticSemaphore_t m_semaphoreBuffer;
    SemaphoreHandle_t m_semaphore;

    std::array<uint8_t*, 1024> m_stackData;
    StaticTask_t m_stackBuffer;
    TaskHandle_t m_taskHandle;

    bool m_isBusy;
};

#endif // __SPIESP_H__
