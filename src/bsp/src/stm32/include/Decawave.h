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

enum class DW_LED { LED_0, LED_1, LED_2, LED_3 };
enum class UWBSpeed { SPEED_110K, SPEED_850K, SPEED_6M8 };

class Decawave {
  public:
    Decawave(decaDevice_t spiDevice);
    Decawave(decaDevice_t spiDevice, uint8_t channel, UWBSpeed speed);
    ~Decawave() = default;

    bool start();

    void setLed(DW_LED led, bool enabled);
    bool setChannel(uint8_t channelNo);
    void setSpeed(UWBSpeed speed);

    void receiveAsync(UWBRxFrame& frame, uint16_t timeoutUs);
    void receive(UWBRxFrame& frame, uint16_t timeoutUs);

    bool transmit(uint8_t* buf, uint16_t length);
    bool transmit(uint8_t* buf, uint16_t length, uint8_t flags);
    bool transmitDelayed(uint8_t* buf, uint16_t length, uint64_t txTimestamp);

    friend void rxCallback(const dwt_cb_data_t* callbackData, void* context);
    friend void isrCallback(void* context);
    friend void rxAsyncTask(void* context);

  private:
    decaDevice_t m_spiDevice;
    dwt_config_t m_dwConfig;

    uint8_t m_channelNo;
    UWBSpeed m_speed;

    TaskHandle_t m_rxTaskHandle;
    dwt_cb_data_t m_callbackData;

    BaseTask<configMINIMAL_STACK_SIZE> m_rxAsyncTask;
    UWBRxFrame* m_rxFrame;

    std::array<uint8_t, UWB_MAX_LENGTH + 2> m_txBuffer;

    void configureDW();
    void retrieveRxFrame(UWBRxFrame* frame);
};

#endif //__DECAWAVE_H__
