set(LIB_NAME "swarmus-hivemind-bsp-stm32f429zi")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP::STM32F4")

set(LIB_SOURCES
    Core/Src/stm32f4xx_hal_msp.c
    Core/Src/system_stm32f4xx.c
    Core/Src/stm32f4xx_it.c
    Core/Src/usart.c
    Core/Src/main.c
    Core/Src/gpio.c
)

set(LIB_HEADERS
    Core/Inc/stm32f4xx_hal_conf.h
    Core/Inc/stm32f4xx_it.h
    Core/Inc/usart.h
    Core/Inc/main.h
    Core/Inc/gpio.h
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


add_library(${LIB_NAME}
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
        STM32::NoSys
        HAL::STM32::F4::RCC
        HAL::STM32::F4::GPIO
        HAL::STM32::F4::CORTEX
)

# Removing warnings and compile options
target_compile_options(${LIB_NAME}
    PRIVATE
        -w
)

# Provide alias for linker
set_target_properties(CMSIS::STM32::F429ZI PROPERTIES IMPORTED_GLOBAL TRUE)
add_library(SwarmUS::HiveMind::BSP::STM32F4::Linker ALIAS CMSIS::STM32::F429ZI)

