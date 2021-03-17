#ifndef HIVE_MIND_USB_H
#define HIVE_MIND_USB_H

#include "bsp/IUSB.h"
#include <BaseTask.h>
#include <c-common/circular_buff.h>
#include <cstdint>
#include <logger/ILogger.h>

class USB : public IUSB {
  public:
    USB(ILogger& logger);
    ~USB() override = default;

    bool send(const uint8_t* buffer, uint16_t length) override;
    bool receive(uint8_t* buffer, uint16_t length) override;
    bool isConnected() override;

  private:
    ILogger& m_logger;
    TaskHandle_t m_receivingTaskHandle;
};

#endif // HIVE_MIND_USB_H
