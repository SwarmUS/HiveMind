#include "hal/hal_gpio.h"

static void* espCallBackInstance = NULL;
static gpioCallbackFct espCallBackFct = NULL;

void setEspCallback(gpioCallbackFct fct, void* instance) {
    if (fct != NULL && instance != NULL) {
        espCallBackFct = fct;
        espCallBackInstance = instance;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

    if (GPIO_Pin == WROOM_USER0_Pin) {
        if (espCallBackFct != NULL && espCallBackInstance != NULL) {
            espCallBackFct(espCallBackInstance);
        }
    }
}
