# STM32F429ZI

This is hal layer for the stm32f4 specefic boards

Couple things to note:
There are two libraries provided

* One is the actual hal library with the uart, eth, gpi, etc
* The other is the "linker" library wich include
  * The global symbols (timer, interrupts, etc)
  * The linker options (nosys.spec)
  * The linker script
  * The assembly startup (startup_stm32f4.s)
