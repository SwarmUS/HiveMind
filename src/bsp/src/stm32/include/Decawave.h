#ifndef __DECAWAVE_H__
#define __DECAWAVE_H__

#include "deca_device_api.h"
#include "deca_port.h"
#include <functional>

#define UWB_MAX_LENGTH 125 // 127 - 2 CRC bytes

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

    void receiveAsync(uint8_t* buf, uint16_t length, std::function<void(bool)>);
    bool receive(uint8_t* buf, uint16_t length);

    bool transmit(uint8_t* buf, uint16_t length);
    bool transmit(uint8_t* buf, uint16_t length, uint8_t flags);
    bool transmitDelayed(uint8_t* buf, uint16_t length, uint64_t txTimestamp);

  private:
    decaDevice_t m_spiDevice;
    dwt_config_t m_dwConfig;

    uint8_t m_channelNo;
    UWBSpeed m_speed;

    std::array<uint8_t, UWB_MAX_LENGTH + 2> m_txBuffer;

    void configureDW();
};

#endif //__DECAWAVE_H__
