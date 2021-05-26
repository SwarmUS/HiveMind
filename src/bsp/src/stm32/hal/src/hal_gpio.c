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

void enableESP() { HAL_GPIO_WritePin(WROOM_EN_GPIO_Port, WROOM_EN_Pin, 1); }

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ESP_USER0_Pin) {
        if (espCallBackFct != NULL && espCallBackContext != NULL) {
            espCallBackFct(espCallBackContext);
        }
    } else {
        for (int i = 0; i < DWT_NUM_DW_DEV; i++) {
            decawaveDeviceConfig_t* decaConfig = deca_getDeviceConfig(i);

            if (GPIO_Pin == decaConfig->irqPin) {
                deca_isr(i);
                break;
            }
        }
    }
}
