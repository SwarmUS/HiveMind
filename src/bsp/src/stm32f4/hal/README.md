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
* The System clock is set to TIM1
* The main is not generated
* The Sys