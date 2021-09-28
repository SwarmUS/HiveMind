set(LIB_NAME "swarmus-hivemind-hal-stm32h735zg-platform")
set(LIB_ALIAS "SwarmUS::HiveMind::HAL::STM32::Platform")

set(LIB_SOURCES
        user_interface.c
        tca9539.c
        tca9539.h
        hal_init.c
        hal_flash.c
    )

add_library(${LIB_NAME} INTERFACE)
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_sources(${LIB_NAME} INTERFACE ${LIB_SOURCES})

target_include_directories(${LIB_NAME}
        INTERFACE
        ./
    )
