#ifndef __DECAWAVE_H__
#define __DECAWAVE_H__

#include "UWBRxFrame.h"
#include "deca_device_api.h"
#include "deca_port.h"
#include <BaseTask.h>
#include <FreeRTOS.h>
#include <functional>
#include <memory>
#include <task.h>
//calibration constants
#define TX_ANT_DLY 16505
#define RX_ANT_DLY 16505

#define POLL_TX_TO_RESP_RX_DLY_UUS  300
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
#define RESP_RX_TIMEOUT_UUS 2700



enum class DW_LED { LED_0 = DWT_GxM0, LED_1 = DWT_GxM1, LED_2 = DWT_GxM2, LED_3 = DWT_GxM3 };
enum class UWBSpeed { SPEED_110K = DWT_BR_110K, SPEED_850K = DWT_BR_850K, SPEED_6M8 = DWT_BR_6M8 };
enum class UWBChannel {
    CHANNEL_1 = 1,
    CHANNEL_2 = 2,
    CHANNEL_3 = 3,
    CHANNEL_4 = 4,
    CHANNEL_5 = 5,
    CHANNEL_6 = 6,
    CHANNEL_7 = 7
};

class Decawave {
  public:
    explicit Decawave(decaDevice_t spiDevice);
    Decawave(decaDevice_t spiDevice, UWBChannel channel, UWBSpeed speed);
    ~Decawave() = default;

    /**
     * @brief Tries communicating by SPI with the DW1000 and configures ISR, GPIO, etc.
     * @return true if successful, false otherwise.
     */
    bool init();

    /**
     * @brief Toggles a DW LED
     * @param led The LED to control
     * @param enabled True if ON false if OFF
     */
    void setLed(DW_LED led, bool enabled);

    /**
     * @brief Sets the UWB channel to use
     * @param channel Channel to use
     * @return True if configured, false if invalid channel
     */
    bool setChannel(UWBChannel channel);

    /**
     * @brief Sets the UWB data rate to use
     * @param speed The data rate to use
     */
    void setSpeed(UWBSpeed speed);

    /**
     * @brief Starts listening for a packet and blocks until received
     * @param frame [out] Variable in which all frame data will be put once packet is received
     * @param timeoutUs Timeout in microseconds. 0 to block indefinitely
     */
    void receive(UWBRxFrame& frame, uint16_t timeoutUs);

    /**
     * @brief Starts listening for a packet at a given timestamp and blocks until received
     * @param frame [out] Variable in which all frame data will be put once packet is received
     * @param timeoutUs Timeout in microseconds. 0 to block indefinitely
     * @param rxStartTime DW timestamp at which to start the RX
     */
    void receiveDelayed(UWBRxFrame& frame, uint16_t timeoutUs, uint64_t rxStartTime);

    /**
     * @brief Starts listening for a packet without blocking. The user should poll the frame.status
     * to see if transmission has been received.
     * @param frame [out] Variable in which all frame data will be put once packet is received
     * @param timeoutUs Timeout in microseconds. 0 to block indefinitely
     */
    void receiveAsync(UWBRxFrame& frame, uint16_t timeoutUs);

    /**
     * @brief Starts listening for a packet at a given timestamp without blocking. The user should
     * poll the frame.status to see if transmission has been received.
     * @param frame [out] Variable in which all frame data will be put once packet is received
     * @param timeoutUs Timeout in microseconds. 0 to block indefinitely
     * @param rxStartTime DW timestamp at which to start the RX
     */
    void receiveAsyncDelayed(UWBRxFrame& frame, uint16_t timeoutUs, uint64_t rxStartTime);

    /**
     * @brief Transmits data over UWB
     * @param buf Buffer to transmit
     * @param length Length of the data
     * @return True if successful, false otherwise.
     */
    bool transmit(uint8_t* buf, uint16_t length);

    /**
     * @brief Transmits data over UWB at a given timestamp
     * @param buf Buffer to transmit
     * @param length Length of the data
     * @param txTimestamp DW timestamp at which to start the transmission
     * @return True if successful, false otherwise.
     */
    bool transmitDelayed(uint8_t* buf, uint16_t length, uint64_t txTimestamp);

    /**
     * @brief Transmits data over UWB and blocks until response is received
     * @param buf Buffer to transmit
     * @param length Length of the data
     * @param rxAfterTxTimeUs Number of microseconds between the TX and the start of the RX
     * @param frame [out] Variable in which all frame data will be put once packet is received
     * @param rxTimeoutUs Timeout in microseconds. 0 to block indefinitely
     * @return True if successful, false otherwise.
     */
    bool transmitAndReceive(uint8_t* buf,
                            uint16_t length,
                            uint32_t rxAfterTxTimeUs,
                            UWBRxFrame& frame,
                            uint16_t rxTimeoutUs);

    /**
     * @brief Transmits data over UWB and blocks until response is received
     * @param buf Buffer to transmit
     * @param length Length of the data
     * @param txTimestamp DW timestamp at which to start the transmission
     * @param rxAfterTxTimeUs Number of microseconds between the TX and the start of the RX
     * @param frame [out] Variable in which all frame data will be put once packet is received
     * @param rxTimeoutUs Timeout in microseconds. 0 to block indefinitely
     * @return True if successful, false otherwise.
     */
    bool transmitDelayedAndReceive(uint8_t* buf,
                                   uint16_t length,
                                   uint64_t txTimestamp,
                                   uint32_t rxAfterTxTimeUs,
                                   UWBRxFrame& frame,
                                   uint16_t rxTimeoutUs);

    bool transmitAndReceiveDelayed(uint8_t* buf,
                                             uint16_t length,
                                             uint32_t rxStartDelayUS,
                                             UWBRxFrame& frame,
                                             uint16_t rxTimeoutUs);

    //TEMP
    void setTxAntennaDLY(uint16 delay);
    void setRxAntennaDLY(uint16 delay);

  private:
    decaDevice_t m_spiDevice;
    dwt_config_t m_dwConfig;

    UWBChannel m_channel;
    UWBSpeed m_speed;

    TaskHandle_t m_rxTaskHandle;
    dwt_cb_data_t m_callbackData;

    BaseTask<configMINIMAL_STACK_SIZE> m_rxAsyncTask;
    UWBRxFrame* m_rxFrame;

    std::array<uint8_t, UWB_MAX_LENGTH> m_txBuffer;

    void configureDW();
    bool transmitInternal(uint8_t* buf, uint16_t length, uint8_t flags);

    void receiveInternal(UWBRxFrame& frame,
                         uint16_t timeoutUs,
                         uint8_t flags,
                         bool rxStarted = false);
    void receiveAsyncInternal(UWBRxFrame& frame,
                              uint16_t timeoutUs,
                              uint8_t flags,
                              bool rxStarted = false);

    void retrieveRxFrame(UWBRxFrame* frame);

    static void rxCallback(const dwt_cb_data_t* callbackData, void* context);
    static void isrCallback(void* context);
    static void rxAsyncTask(void* context);
};

#endif //__DECAWAVE_H__