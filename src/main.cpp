#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <task.h>
#include <timers.h>

#include <bsp/bsp.h>
#include <bsp/ui_impl.h>
#include <iostream>

void blinky(void* param) {
    void* param_used = param;
    const int toggle_delay = 500;
    UIImpl ui = UIImpl();
    while (true) {
        vTaskDelay(toggle_delay);
        ui.print("I love boobies");
    }
}

int main() {
    init_chip();
    std::cout << "HELLO WORD";
    xTaskCreate(blinky, "blinky", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL);
    vTaskStartScheduler();
    while (true) {
    };

    return 0;
}
