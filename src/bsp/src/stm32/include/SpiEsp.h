#ifndef __SPIESP_H__
#define __SPIESP_H__

#include "SpiHeader.h"
#include "bsp/ICRC.h"
#include "bsp/ICommInterface.h"
#include "logger/ILogger.h"
#include <BaseTask.h>
#include <Task.h>
#include <array>

#define CRC32_SIZE sizeof(uint32_t)
#define ESP_SPI_MAX_MESSAGE_LENGTH (2048u - CRC32_SIZE)

class SpiEsp : public ICommInterface {
  public:
    SpiEsp(ICRC& crc, ILogger& logger);
    ~SpiEsp() override = default;

    bool send(const uint8_t* buffer, uint16_t length) override;

    bool receive(uint8_t* buffer, uint16_t length) override {
        (void)buffer;
        (void)length;
        return false;
    }

    bool isBusy() const;
    bool isConnected() const override;

    void execute();

  private:
    BaseTask<configMINIMAL_STACK_SIZE * 3> m_driverTask;
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

    EspHeader::Header m_outboundHeader;
    EspHeader::Header* m_inboundHeader;

    static void espInterruptCallback(void* context);
    static void espTxRxCallback(void* context);

    void updateOutboundHeader();
    bool m_inboundRequest;

    bool m_isBusy;
};

#endif // __SPIESP_H__
