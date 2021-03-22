#ifndef HIVE_MIND_USB_H
#define HIVE_MIND_USB_H

#include "LockGuard.h"
#include "bsp/IUSB.h"
#include "semphr.h"
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
    bool isConnected() const override;

  private:
    static void interruptRxCallback(void* context, uint8_t* buffer, uint32_t length);
    void receiveItCallback(uint8_t* buf, uint32_t len);

    ILogger& m_logger;
    TaskHandle_t m_receivingTaskHandle = NULL;
};

#endif // HIVE_MIND_USB_H
