set(LIB_NAME "swarmus-hivemind-hal-stm32h735zg")
set(LIB_ALIAS "SwarmUS::HiveMind::HAL::STM32")

set(LIB_SOURCES
    Core/Src/usart.c
    Core/Src/dma.c
    Core/Src/main.c
    Core/Src/gpio.c
    Core/Src/crc.c
    Core/Src/rng.c
    Core/Src/spi.c
    Core/Src/i2c.c
    Core/Src/tim.c
)

set(LIB_HEADERS
    Core/Inc/stm32h7xx_hal_conf.h
    Core/Inc/stm32h7xx_it.h
    Core/Inc/hivemind_hal.h
    Core/Inc/dma.h
    Core/Inc/usart.h
    Core/Inc/main.h
    Core/Inc/gpio.h
    Core/Inc/crc.h
    Core/Inc/rng.h
    Core/Inc/spi.h
    Core/Inc/i2c.h
    Core/Inc/tim.h
)

file(REMOVE Core/Inc/FreeRTOSConfig.h) #Need to remove since we already provide it in the OS, but can't disable it in cubemx since lwip config depends on it

# Sources that needs to be global on the project
# Ex: interrupts, syscalls, callbacks
set(GLOBAL_SOURCES
    Core/Src/stm32h7xx_it.c
    Core/Src/stm32h7xx_hal_timebase_tim.c
    Core/Src/stm32h7xx_hal_msp.c
)

stm32_fetch_cmsis(H7)
stm32_fetch_hal(H7)
stm32_fetch_cube(H7)

find_package(CMSIS COMPONENTS
            STM32H735ZG_M7
            REQUIRED)

find_package(HAL COMPONENTS 
            STM32H735ZG_M7
            REQUIRED)

# Creating the interface library
add_library(stm32h735zg-linker INTERFACE)
add_library(${LIB_HAL_LINKER_ALIAS} ALIAS stm32h735zg-linker) # Provide alias for linker options

# Defining entry point
set(LIB_SOURCES
    startup_stm32f429xx.s
    Core/Src/system_stm32f4xx.c
)
add_library(Swarmus::CMSIS::Startup INTERFACE IMPORTED)
target_link_libraries(Swarmus::CMSIS::Startup INTERFACE CMSIS::STM32::H7 STM32::H735xx::M7)
target_sources(Swarmus::CMSIS::Startup INTERFACE ${LIB_SOURCES})


target_sources(stm32h735zg-linker
    INTERFACE
        ${GLOBAL_SOURCES})

target_include_directories(stm32h735zg-linker
    INTERFACE
        Core/Inc
        LWIP/App
        LWIP/Target
        USB_DEVICE/App
        USB_DEVICE/Target
        Drivers/Include
)

target_link_libraries(stm32h735zg-linker
    INTERFACE
        STM32::H7::M7
        Swarmus::CMSIS::Startup
        HAL::STM32::H7::M7
        STM32::NoSys
        SwarmUS::HiveMind::LWIP
        CMSIS::STM32::H7::M7::RTOS_V2
    )
# linker file
configure_file(H735ZG_M7.in.ld H735ZG_M7.ld)
set_default(USER_DATA_FLASH_SIZE "0K")
stm32_add_linker_script(stm32h735zg-linker INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/H735ZG_M7.ld)

# Actual bsp library
add_library(${LIB_NAME} 
    STATIC
        ${LIB_SOURCES}
        ${LIB_HEADERS}
)

add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})
        
target_include_directories(${LIB_NAME} 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        Core/Inc

        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        LWIP/App

        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        USB_DEVICE/App
)

target_link_libraries(${LIB_NAME}
    PUBLIC 
        Swarmus::CMSIS::Startup
        HAL::STM32::H7::M7::CORTEX
        STM32::NoSys
        SwarmUS::Propolis::C::Common
        SwarmUS::HiveMind::HAL::STM32H7::USB

    PRIVATE
        stm32h735zg-linker

        # Drivers
        HAL::STM32::H7::M7::TIM
        HAL::STM32::H7::M7::TIMEx
        HAL::STM32::H7::M7::RNG
        HAL::STM32::H7::M7::SPI
        HAL::STM32::H7::M7::FLASH
        HAL::STM32::H7::M7::FLASHEx
        HAL::STM32::H7::M7::GPIO
        HAL::STM32::H7::M7::DMA
        HAL::STM32::H7::M7::CRC
        HAL::STM32::H7::M7::CRCEx
        HAL::STM32::H7::M7::SPI
        HAL::STM32::H7::M7::UART
        HAL::STM32::H7::M7::UARTEx
        HAL::STM32::H7::M7::RCC
        HAL::STM32::H7::M7::RCCEx
        HAL::STM32::H7::M7::PWREx
        HAL::STM32::H7::M7::I2C
        HAL::STM32::H7::M7::I2CEx
        CMSIS::STM32::H7::M7::RTOS_V2
)

add_subdirectory(LWIP)
add_subdirectory(USB_DEVICE)

# Removing warnings and compile options
if (DISABLE_EXTERNAL_WARNINGS) 
    target_compile_options(${LIB_NAME} PRIVATE -w)
    set_target_properties(${LIB_NAME} PROPERTIES CXX_CLANG_TIDY "" )
    set_target_properties(${LIB_NAME} PROPERTIES C_CLANG_TIDY "" )
endif()
