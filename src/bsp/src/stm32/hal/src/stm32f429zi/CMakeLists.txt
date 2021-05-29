set(LIB_NAME "swarmus-hivemind-hal-stm32f429zi-platform")
set(LIB_ALIAS "SwarmUS::HiveMind::HAL::STM32::Platform")

set(LIB_SOURCES
        user_interface.c
    )


add_library(${LIB_NAME} INTERFACE)
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_sources(${LIB_NAME} INTERFACE ${LIB_SOURCES})

target_include_directories(${LIB_NAME}
        INTERFACE
        ./
    )
