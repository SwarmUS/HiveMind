set(LIB_NAME "swarmus-hivemind-bsp-stm32f429zi")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP::STM32F4")

set(LIB_SOURCES
    Core/Src/stm32f4xx_hal_msp.c
    # Core/Src/system_stm32f4xx.c Already provided by CMSIS, no need for it
    Core/Src/usart.c
    Core/Src/main.c
    Core/Src/gpio.c
)

set(LIB_HEADERS
    Core/Inc/stm32f4xx_hal_conf.h
    Core/Inc/stm32f4xx_it.h
    Core/Inc/hivemind_hal.h
    Core/Inc/usart.h
    Core/Inc/main.h
    Core/Inc/gpio.h
)

# Sources that needs to be global on the project
# Ex: interrupts, syscalls
set(GLOBAL_SOURCES
    Core/Src/stm32f4xx_it.c
    Core/Src/stm32f4xx_hal_timebase_tim.c
    Core/Src/syscalls.c
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
add_library(stm32f4-linker INTERFACE)
add_library(${LIB_BSP_LINKER_ALIAS} ALIAS stm32f4-linker) # Provide alias for linker options
set_target_properties(CMSIS::STM32::F429ZI PROPERTIES IMPORTED_GLOBAL TRUE)

target_sources(stm32f4-linker
    INTERFACE
        ${GLOBAL_SOURCES})

target_include_directories(stm32f4-linker
    INTERFACE
        Core/Inc
)

target_link_libraries(stm32f4-linker
    INTERFACE
        CMSIS::STM32::F429ZI
        HAL::STM32::F4
        HAL::STM32::F4::TIM
        HAL::STM32::F4::TIMEx
        STM32::NoSys)

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
)

target_link_libraries(${LIB_NAME}
    PUBLIC 
        CMSIS::STM32::F429ZI
        HAL::STM32::F4::CORTEX
        STM32::NoSys

    PRIVATE
        stm32f4-linker

        # Drivers
        HAL::STM32::F4::RCC
        HAL::STM32::F4::GPIO
        HAL::STM32::F4::UART
        HAL::STM32::F4::DMA

)

# Removing warnings and compile options
if(DISABLE_EXTERNAL_WARNINGS)
    target_compile_options(${LIB_NAME}
        PRIVATE
            -w
    )
    set_target_properties(${LIB_NAME} PROPERTIES CXX_CLANG_TIDY "")
endif()


