freertos_fetch_kernel()
stm32_fetch_cmsis(F4)
stm32_fetch_hal(F4)
stm32_fetch_cube(F4)


find_package(CMSIS COMPONENTS STM32F407VG REQUIRED)
find_package(HAL COMPONENTS STM32F407VG REQUIRED)
find_package(FreeRTOS COMPONENTS ARM_CM4F REQUIRED)


add_subdirectory(freertos)
add_subdirectory(bsp)

set(PROJECT_SOURCES
    main.c
)

add_executable(stm32-freertos.elf ${PROJECT_SOURCES})
target_link_libraries(stm32-freertos.elf PRIVATE
    # Provided libraries
    FreeRTOS::Timers
    FreeRTOS::Heap::1
    FreeRTOS::ARM_CM4F 
    HAL::STM32::F4::RCC
    HAL::STM32::F4::GPIO
    HAL::STM32::F4::CORTEX
    CMSIS::STM32::F407VG 
    STM32::NoSys

    # Defined libraries
    SwarmUS::HiveMind::FreeRTOS
)
