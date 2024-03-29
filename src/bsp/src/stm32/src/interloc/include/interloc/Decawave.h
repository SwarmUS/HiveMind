#ifndef __DECAWAVE_H__
#define __DECAWAVE_H__

#include "DecawaveUtils.h"
#include "UWBRxFrame.h"
#include "deca_device_api.h"
#include "deca_port.h"
#include <BaseTask.h>
#include <FreeRTOS.h>
#include <functional>
#include <memory>
#include <task.h>
// calibration constants
#define DEFAULT_TX_ANT_DLY 16505U // 247ns
#define DEFAULT_RX_ANT_DLY 16505U

#define UUS_TO_DWT_TIME 63898ULL
#define SPEED_OF_LIGHT 299792458ULL
#define DW_INTERNAL_CLOCK_RFEQ 63897600000

// Time management constants
#define SPI_SPEED_HZ 2625000U // bits/s
#define PHY_HEADER_LENGTH 21U // symbols

#define DW_CHANNEL CHANNEL_2
#define DW_SPEED SPEED_6M8

enum class DW_LED { LED_0 = DWT_GxM0, LED_1 = DWT_GxM1, LED_2 = DWT_GxM2, LED_3 = DWT_GxM3 };
enum class DW_SYNC_MODE { OSTR, OFF };
enum class DW_STATE { CONFIGURED, SEND_CALIB, RESPOND_CALIB, CALIBRATED };

class Decawave {
  public:
    explicit Decawave(decaDevice_t spiDevice);
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
     * @brief Starts listening for a packet and blocks until received. Once received, the packet
     * data can be retrieved with retrieveRxFrame().
     * @param timeoutUs Timeout in microseconds. 0 to block indefinitely
     * @return The status of the reception.
     */
    UWBRxStatus receive(uint16_t timeoutUs);

    /**
     * @brief Starts listening for a packet at a given timestamp and blocks until received. Once
     * received, the packet data can be retrieved with retrieveRxFrame().
     * @param timeoutUs Timeout in microseconds. 0 to block indefinitely
     * @param rxStartTime DW timestamp at which to start the RX
     * @return The status of the reception.
     */
    UWBRxStatus receiveDelayed(uint16_t timeoutUs, uint64_t rxStartTime);

    /**
     * @brief Starts listening for a packet without blocking. The user should poll getRxStatus()
     * to see if transmission has been received.
     * @param timeoutUs Timeout in microseconds. 0 to block indefinitely
     */
    void receiveAsync(uint16_t timeoutUs);

    /**
     * @brief Starts listening for a packet at a given timestamp without blocking. The user should
     * poll getRxStatus() to see if transmission has been received.
     * @param timeoutUs Timeout in microseconds. 0 to block indefinitely
     * @param rxStartTime DW timestamp at which to start the RX
     */
    void receiveAsyncDelayed(uint16_t timeoutUs, uint64_t rxStartTime);

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
     * @param rxTimeoutUs Timeout in microseconds. 0 to block indefinitely
     * @return An UWBRxStatus if successful, false otherwise.
     */
    std::optional<UWBRxStatus> transmitAndReceive(uint8_t* buf,
                                                  uint16_t length,
                                                  uint32_t rxAfterTxTimeUs,
                                                  uint16_t rxTimeoutUs);

    /**
     * @brief Transmits data over UWB and blocks until response is received
     * @param buf Buffer to transmit
     * @param length Length of the data
     * @param txTimestamp DW timestamp at which to start the transmission
     * @param rxAfterTxTimeUs Number of microseconds between the TX and the start of the RX
     * @param rxTimeoutUs Timeout in microseconds. 0 to block indefinitely
     * @return An UWBRxStatus if successful, false otherwise.
     */
    std::optional<UWBRxStatus> transmitDelayedAndReceive(uint8_t* buf,
                                                         uint16_t length,
                                                         uint64_t txTimestamp,
                                                         uint32_t rxAfterTxTimeUs,
                                                         uint16_t rxTimeoutUs);
    /**
     * @brief Transmits data over UWB and blocks until response is received
     * @param buf Buffer to transmit
     * @param length Length of the data
     * @param rxStartDelayUS Number of microseconds between the TX and the start of the RX
     * @param rxTimeoutUs Timeout in microseconds. 0 to block indefinitely
     * @return An UWBRxStatus if successful, false otherwise.
     */
    std::optional<UWBRxStatus> transmitAndReceiveDelayed(uint8_t* buf,
                                                         uint16_t length,
                                                         uint32_t rxStartDelayUS,
                                                         uint16_t rxTimeoutUs);

    /**
     * Prepares the DW1000 for a clock sync
     * @param syncMode Sync mode to prepare for
     */
    void setSyncMode(DW_SYNC_MODE syncMode);

    /**
     * @brief Sets transmission antenna delay in DecawaveTimeUnits
     * @param delay Delay in DecawaveTimeUnits
     */
    void setTxAntennaDLY(uint16 delay);

    /**
     * @brief Sets reception antenna delay in DecawaveTimeUnits
     * @param delay Delay in DecawaveTimeUnits
     */
    void setRxAntennaDLY(uint16 delay);

    /**
     * @brief Get transmission antenna delay in DecawaveTimeUnits
     * @return Delay in DecawaveTimeUnits
     */
    uint16_t getTxAntennaDLY() const;

    /**
     * @brief Get reception antenna delay in DecawaveTimeUnits
     * @return Delay in DecawaveTimeUnits
     */
    uint16_t getRxAntennaDLY() const;

    /**
     * @brief Retrieves transmission timestamp in DecawaveTimeUnits
     * @param txTimestamp Transmission timestamp in DecawaveTimeUnits
     */
    void getTxTimestamp(uint64_t* txTimestamp);

    /**
     * @brief Retrieves reception timestamp in DecawaveTimeUnits
     * @param rxTimestamp Reception timestamp in DecawaveTimeUnits
     */
    void getRxTimestamp(uint64_t* rxTimestamp) const;

    /**
     * @brief Retrieves present time DecawaveTimeUnits
     * @return Present timestamp in DecawaveTimeUnits
     */
    uint64_t getSysTime();

    /**
     * @brief Returns the system time (in DTUs) at which a message will be sent when a delayed tx is
     * called with the argument
     * @param txTime The time used to call the delayed TX
     * @return The SysTime at which the packet will be sent
     */
    uint64_t getTxTimestampFromDelayedTime(uint64_t txTime) const;

    /**
     * @brief Retrieves the present state of calibration
     * @return The present state of calibration
     */
    DW_STATE getState();

    /**
     * @brief Sets the next state of calibration
     * @param state The next state of calibration
     */
    void setState(DW_STATE state);

    /**
     * Tells if the specified DW is ready (AKA was successfully configured)
     * @return Readiness flag
     */
    bool isReady() const;

    /**
     * @brief Retrieves RX frame information from the Decawave's internal registers
     * @param frame A reference to the frame in which to save the information
     */
    void retrieveRxFrame(UWBRxFrame& frame) const;

    /**
     * @brief Returns the status of the last RX operation
     * @return Status
     */
    UWBRxStatus getRxStatus() const;

    /**
     * @brief Blocks until the ongoing RX operation is finished, returning the RxStatus
     * @return Status of the finished RX operation
     */
    UWBRxStatus awaitRx();

    /**
     * Aborts any ongoing RX or TX operation (to be called from ISR)
     */
    void abortTRXFromISR();

  private:
    decaDevice_t m_spiDevice;
    dwt_config_t m_dwConfig;

    UWBChannel m_channel;
    UWBSpeed m_speed;
    uint16_t m_rxAntennaDelayDTU = DEFAULT_RX_ANT_DLY;
    uint16_t m_txAntennaDelayDTU = DEFAULT_TX_ANT_DLY;

    TaskHandle_t m_trxTaskHandle;
    dwt_cb_data_t m_callbackData;

    UWBRxFrame* m_rxFrame;

    std::array<uint8_t, UWB_MAX_LENGTH> m_txBuffer;

    DW_STATE m_state;

    UWBRxStatus m_rxStatus;

    bool m_isReady;

    void configureDW();
    bool transmitInternal(uint8_t* buf, uint16_t length, uint8_t flags);

    UWBRxStatus receiveInternal(uint16_t timeoutUs, uint8_t flags, bool rxStarted = false);
    void receiveAsyncInternal(uint16_t timeoutUs, uint8_t flags, bool rxStarted = false);

    static void rxCallback(const dwt_cb_data_t* callbackData, void* context);
    static void txCallback(const dwt_cb_data_t* callbackData, void* context);
    static void isrCallback(void* context);
    void enableAccumulatorClock() const;
};
// user manual : https://www.decawave.com/sites/default/files/resources/dw1000_user_manual_2.11.pdf
// datasheet : https://www.decawave.com/sites/default/files/resources/dw1000-datasheet-v2.09.pdf

#endif //__DECAWAVE_H__