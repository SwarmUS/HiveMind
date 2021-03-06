set_default(UUID_OVERRIDE 0)
set_default(HOST_PORT 55551)
set_default(HOST_IP "192.168.1.101")
set_default(LOG_LEVEL "Info")

if (${COMPILE_STM32_F429ZI})
    set(USER_DATA_FLASH_SIZE "128K")
    set(USER_DATA_FLASH_SECTOR "23")
    set(USER_DATA_FLASH_START_ADDRESS "0x081E0000")
elseif (${COMPILE_STM32_H735ZG})
    set(USER_DATA_FLASH_SIZE "128K")
    set(USER_DATA_FLASH_SECTOR "7")
    set(USER_DATA_FLASH_START_ADDRESS "0x080E0000")
else()
    message(FATAL_ERROR "FLASH: The current board is not supported")
endif()
