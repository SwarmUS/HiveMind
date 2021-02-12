#ifndef HAl_GPIO_H
#define HAl_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gpio.h"

typedef void (*gpioCallbackFct)(void* instance);

void setEspCallback(gpioCallbackFct fct, void* instance);

#ifdef __cplusplus
}
#endif

#endif // HAl_GPIO_H
