#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>

#include <task.h>
#include <timers.h>

#include <bsp/bsp.h>
#include <bsp/ui_impl.h>
#include <logger/logger_impl.h>

void print_thread_example(void* param) {
    void* param_used = param;
    const int toggle_delay = 500;
    UIImpl ui = UIImpl();
    LoggerImpl logger = LoggerImpl(LogLevel::Debug, &ui);
    logger.log(LogLevel::Info, "Hello logger!");
    while (true) {
        vTaskDelay(toggle_delay);
        ui.print("Hello world!");
    }
}

int main() {
    init_chip();
    xTaskCreate(print_thread_example, "print", configMINIMAL_STACK_SIZE * 4, NULL,
                tskIDLE_PRIORITY + 1, NULL);
    vTaskStartScheduler();
    while (true) {
    };

    return 0;
}
