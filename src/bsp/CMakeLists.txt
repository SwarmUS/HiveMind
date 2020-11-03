set(LIB_NAME "swarmus-hivemind-bsp")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP")


set(LIB_SOURCES
    src/stm32f4/bsp.cpp
)

set(LIB_HEADERS
  include/bsp.h
)

freertos_fetch_kernel()
stm32_fetch_cmsis(F4)
stm32_fetch_hal(F4)
stm32_fetch_cube(F4)

find_package(CMSIS COMPONENTS STM32F407VG REQUIRED)
find_package(HAL COMPONENTS STM32F407VG REQUIRED)
find_package(FreeRTOS COMPONENTS ARM_CM4F REQUIRED)

add_library(${LIB_NAME}  ${LIB_SOURCES} ${LIB_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
    PUBLIC
       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
       $<INSTALL_INTERFACE:include>
       include/stm32f4
    PRIVATE
       src
  )


target_link_libraries(${LIB_NAME}
    PUBLIC
        # Provided libraries
        FreeRTOS::Timers
        FreeRTOS::Heap::1
        FreeRTOS::ARM_CM4F 

        # Defined libraries
        SwarmUS::HiveMind::FreeRTOS

    PRIVATE
        # Provided libraries
        HAL::STM32::F4::RCC
        HAL::STM32::F4::GPIO
        HAL::STM32::F4::CORTEX
        CMSIS::STM32::F407VG 
        STM32::NoSys
)

# TMP
target_compile_options(${LIB_NAME} PUBLIC 
    -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
)
target_link_options(${LIB_NAME} PUBLIC 
    -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
)
