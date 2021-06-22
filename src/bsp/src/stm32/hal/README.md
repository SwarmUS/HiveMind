# STM32F4

This is hal layer for the stm32f4 specefic boards

Couple things to note:
There are two libraries provided

One is the actual hal library with the uart, eth, gpi, etc
The hal library needs to provide a hivemind_hal.h file, all common use definition of varaible will be there (uart used to print, status led, etc)

The other is the "linker" library wich include
* The global symbols (timer, interrupts, etc)
* The linker options (nosys.spec)
* The linker script
* The assembly startup (startup_stm32f4.s)


What was done
* The project is generated usgin GPDSC, just because it generates less files
* The System clock is set to TIM1
* The main function is not generated
* The library files are used as reference, they don't copy in the project
* The SysClock config function was added to the main.h
* Some functions are not generated in NVIC->Code generation since they are provided by FreeRTOS
    * System tick timer (SysTick_IRQn)
    * Pendable request for system services (PenSV_IRQn)
    * System service  call via SWI function (SVCall_IRQn)
* A bit of code was added to the drivers
    * Added __io_putchar definition
    * Added syscall, you can probably just copy/paste