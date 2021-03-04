//
// Created by hubert on 3/4/21.
//

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

class USB : public IUSB {
  public:
    explicit USB(ICRC& crc, ILogger& logger);
    ~USB() override = default;

    bool send(const uint8_t* buffer, uint16_t length) override;
    bool receive(uint8_t* buffer, uint16_t length) override;

#endif // HIVE_MIND_USB_H
