set(LIB_NAME "swarmus-hivemind-hal-stm32f429zi")
set(LIB_ALIAS "SwarmUS::HiveMind::HAL::STM32")


set(LIB_SOURCES
    # Core/Src/system_stm32f4xx.c Already provided by CMSIS, no need for it
    Core/Src/usart.c
    Core/Src/main.c
    Core/Src/gpio.c
    Core/Src/dma.c
    Core/Src/crc.c
    Core/Src/rng.c
    Core/Src/spi.c
    LWIP/App/lwip.c
    LWIP/Target/ethernetif.c
    USB_DEVICE/App/usbd_cdc_if.c
)

set(LIB_HEADERS
    Core/Inc/stm32f4xx_hal_conf.h
    Core/Inc/stm32f4xx_it.h
    Core/Inc/hivemind_hal.h
    Core/Inc/usart.h
    Core/Inc/main.h
    Core/Inc/gpio.h
    Core/Inc/dma.h
    Core/Inc/crc.h
    Core/Inc/rng.h
    Core/Inc/spi.h
    LWIP/App/lwip.h
    USB_DEVICE/App/usbd_cdc_if.h
)

# Sources that needs to be global on the project
# Ex: interrupts, syscalls, callbacks
set(GLOBAL_SOURCES
    Core/Src/stm32f4xx_it.c
    Core/Src/stm32f4xx_hal_timebase_tim.c
    Core/Src/stm32f4xx_hal_msp.c
)

stm32_fetch_cmsis(F4)
stm32_fetch_hal(F4)
stm32_fetch_cube(F4)

find_package(CMSIS COMPONENTS
            STM32F429ZI
            REQUIRED)

find_package(HAL COMPONENTS 
            STM32F429ZI
            REQUIRED)

# Creating the interface library
add_library(stm32f429zi-linker INTERFACE)
add_library(${LIB_HAL_LINKER_ALIAS} ALIAS stm32f429zi-linker) # Provide alias for linker options
set_target_properties(CMSIS::STM32::F429ZI PROPERTIES IMPORTED_GLOBAL TRUE)

target_sources(stm32f429zi-linker
    INTERFACE
        ${GLOBAL_SOURCES})

target_include_directories(stm32f429zi-linker
    INTERFACE
        Core/Inc
        LWIP/App
        LWIP/Target
        USB_DEVICE/App
        USB_DEVICE/Target
)

target_link_libraries(stm32f429zi-linker
    INTERFACE
        STM32::F4
        CMSIS::STM32::F429ZI
        HAL::STM32::F4
        HAL::STM32::F4::TIM
        HAL::STM32::F4::TIMEx
        STM32::NoSys
        SwarmUS::HiveMind::LWIP
        CMSIS::STM32::F4::RTOS_V2
    )

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
        CMSIS::STM32::F429ZI
        HAL::STM32::F4::CORTEX
        STM32::NoSys
        SwarmUS::Propolis::C::Common
        SwarmUS::HiveMind::HAL::STM32F4::USB

    PRIVATE
        stm32f429zi-linker

        # Drivers
        HAL::STM32::F4::TIM
        HAL::STM32::F4::TIMEx
        HAL::STM32::F4::RNG
        HAL::STM32::F4::SPI
        HAL::STM32::F4::FLASH
        HAL::STM32::F4::FLASHEx
        HAL::STM32::F4::RCC
        HAL::STM32::F4::GPIO
        HAL::STM32::F4::UART
        HAL::STM32::F4::DMA
        HAL::STM32::F4::CRC
        HAL::STM32::F4::SPI
        SwarmUS::HiveMind::LWIP
        CMSIS::STM32::F4::RTOS_V2
)

add_subdirectory(LWIP)
add_subdirectory(USB_DEVICE)

# Removing warnings and compile options
if (DISABLE_EXTERNAL_WARNINGS) 
    target_compile_options(${LIB_NAME} PRIVATE -w)
    set_target_properties(${LIB_NAME} PROPERTIES CXX_CLANG_TIDY "" )
    set_target_properties(${LIB_NAME} PROPERTIES C_CLANG_TIDY "" )
endif()
