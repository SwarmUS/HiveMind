#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hivemind_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

void decawave_reset();

void decawave_wakeup();

void decawave_setSlowRate();

void decawave_setFastRate();

#ifdef __cplusplus
}
#endif

#endif // PORT_H