#ifndef HAl_GPIO_H
#define HAl_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gpio.h"

typedef void (*gpioCallbackFct_t)(void* instance);

void setEspCallback(gpioCallbackFct_t fct, void* instance);
void setResetSendBtnCallback(void (*fct)(void* context), void* context);
void setResetReceiveBtnCallback(void (*fct)(void* context), void* context);
#ifdef __cplusplus
}
#endif

#endif // HAl_GPIO_H
