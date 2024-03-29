#ifndef __SPIESP_H__
#define __SPIESP_H__

#include "SpiHeader.h"
#include "bsp/ICRC.h"
#include "bsp/ICommInterface.h"
#include "logger/ILogger.h"
#include <BaseTask.h>
#include <Task.h>
#include <array>
#include <c-common/circular_buff.h>

constexpr uint8_t CRC32_SIZE = sizeof(uint32_t);
constexpr uint16_t ESP_SPI_MAX_MESSAGE_LENGTH = (512U - CRC32_SIZE);

class SpiEsp : public ICommInterface {
  public:
    SpiEsp(ICRC& crc, ILogger& logger);
    ~SpiEsp() override = default;

    bool send(const uint8_t* buffer, uint16_t length) override;

    bool receive(uint8_t* buffer, uint16_t length) override;

    bool isConnected() const override;

    ConnectionType getType() const override;

    void execute();

  private:
    BaseTask<configMINIMAL_STACK_SIZE * 10> m_driverTask;
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
        uint16_t m_payloadSize;
    } m_inboundMessage, m_outboundMessage;

    std::array<uint8_t, 4 * ESP_SPI_MAX_MESSAGE_LENGTH> m_data;
    CircularBuff m_circularBuf;
    TaskHandle_t m_receivingTaskHandle, m_sendingTaskHandle = nullptr;
    EspHeader::Header m_outboundHeader;
    EspHeader::Header* m_inboundHeader;

    static void espInterruptCallback(void* context);
    static void espTxRxCallback(void* context);

    void updateOutboundHeader();
    bool m_inboundRequest;

    bool m_crcOK;
    volatile bool m_hasSentPayload; // changed in ISR
    bool m_isConnected;
};

#endif // __SPIESP_H__
