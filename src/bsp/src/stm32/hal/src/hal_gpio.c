#include "hal/hal_gpio.h"
#include "deca_port.h"

static void* espCallBackContext = NULL;
static gpioCallbackFct_t espCallBackFct = NULL;

void setEspCallback(gpioCallbackFct_t fct, void* instance) {
    if (fct != NULL && instance != NULL) {
        espCallBackFct = fct;
        espCallBackContext = instance;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

    if (GPIO_Pin == ESP_USER0_Pin) {
        if (espCallBackFct != NULL && espCallBackContext != NULL) {
            espCallBackFct(espCallBackContext);
        }
    } else if (GPIO_Pin == DW_IRQn_A_Pin) {
        deca_isr(DW_A);
    } else if (GPIO_Pin == DW_IRQn_B_Pin) {
        deca_isr(DW_B);
    }
}
