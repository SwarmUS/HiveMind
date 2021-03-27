#include "hal/hal_gpio.h"
#include "deca_port.h"

static void* espCallBackContext = NULL;
static gpioCallbackFct_t espCallBackFct = NULL;
static uint8_t calib_flag = 1;

void (*resetSendCallback)();
void* resetSendCallbackContext;

void (*resetReceiveCallback)();
void* resetReceiveCallbackContext;

void setEspCallback(gpioCallbackFct_t fct, void* instance) {
    if (fct != NULL && instance != NULL) {
        espCallBackFct = fct;
        espCallBackContext = instance;
    }
}

void setResetSendBtnCallback(void (*fct)(void* context), void* context) {
    resetSendCallback = fct;
    resetSendCallbackContext = context;
}

void setResetReceiveBtnCallback(void (*fct)(void* context), void* context) {
    resetReceiveCallback = fct;
    resetReceiveCallbackContext = context;
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
    }else if (GPIO_Pin == USER_Btn_Pin){
        if(calib_flag){
            calib_flag = 0;
            resetSendCallback(resetSendCallbackContext);
            for(int i = 0; i < 100000; i++){}
        }else{
            calib_flag = 1;
            resetReceiveCallback(resetReceiveCallbackContext);
            for(int i = 0; i < 100000; i++){}

        }
        // TODO
        //blink a decawave LED
    }
}
