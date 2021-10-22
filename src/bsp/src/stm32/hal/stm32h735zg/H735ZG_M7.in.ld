ENTRY(Reset_Handler)

_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM); /* end of "RAM" Ram type memory */
_Min_Heap_Size = 0x600;
_Min_Stack_Size = 0x400;

MEMORY
{
    FLASH (rx)      : ORIGIN = 0x8000000, LENGTH = 1024K-${USER_DATA_FLASH_SIZE}
    DTCMRAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K
    RAM_D1 (xrw)       : ORIGIN = 0x24000000, LENGTH = 320k  
    RAM_D2 (xrw)    : ORIGIN = 0x30000000, LENGTH = 288K
}

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  _sidata = LOADADDR(.data);

  .data : 
  {
    . = ALIGN(4);
    _sdata = .; 
    *(.data)
    *(.data*)

    . = ALIGN(4);
    _edata = .;
  } >RAM_D1 AT> FLASH

  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM_D1

  .tcmbss :
  {
    . = ALIGN(4);
    _stcmbss = .;
    *(.tcmbss)
    *(.tcmbss*)
    . = ALIGN(4);
    *(.user_cm)
    *(.user_cm*)
    . = ALIGN(4);
    _etcmbss = .;
  } >DTCMRAM

  .tcmdata :
  {
    . = ALIGN(4);
    _stcmdata = .;
    *(.tcmdata)
    *(.tcmdata*)
    . = ALIGN(4);
    *(.user_cmd_data)
    *(.user_cmd_data*)
    . = ALIGN(4);
    _etcmdata = .;
  } >DTCMRAM AT> FLASH

  _sitcmdata = LOADADDR(.tcmdata);

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >DTCMRAM

  .lwip_sec (NOLOAD) : {
    . = ABSOLUTE(0x30000000);
    *(.RxDecripSection)
    
    . = ABSOLUTE(0x30000060);
    *(.TxDecripSection)

    . = ABSOLUTE(0x30000200);
    *(.RxArraySection)
  } >RAM_D2 AT> FLASH
  

  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
