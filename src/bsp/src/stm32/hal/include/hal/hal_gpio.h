#ifndef HAl_GPIO_H
#define HAl_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gpio.h"

typedef void (*gpioCallbackFct_t)(void* instance);

void setEspCallback(gpioCallbackFct_t fct, void* instance);

void enableESP();
#ifdef __cplusplus
}
#endif

#endif // HAl_GPIO_H
