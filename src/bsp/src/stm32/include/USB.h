#ifndef HIVE_MIND_USB_H
#define HIVE_MIND_USB_H

#include "bsp/ICRC.h"
#include "bsp/IUSB.h"
#include <BaseTask.h>
#include <FreeRTOS.h>
#include <Mutex.h>
#include <array>
#include <c-common/circular_buff.h>
#include <cstdint>
#include <logger/ILogger.h>

#define USB_STREAM_SIZE 4096

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
