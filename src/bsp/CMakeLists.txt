set(LIB_NAME "swarmus-hivemind-bsp")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP")


if (${COMPILE_STM32})
    set(LIB_SOURCES
        src/stm32f4/bsp.cpp
    )
else()
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
    set(LIB_SOURCES
        src/posix/bsp.cpp
    )
endif()

set(LIB_HEADERS
  include/bsp.h
)


freertos_fetch_kernel()
stm32_fetch_cmsis(F4)
stm32_fetch_hal(F4)
stm32_fetch_cube(F4)


find_package(CMSIS COMPONENTS
            STM32F407VG 
            REQUIRED)

find_package(HAL COMPONENTS 
            STM32F407VG
            REQUIRED)

find_package(FreeRTOS COMPONENTS
            ARM_CM4F
            REQUIRED)

add_library(${LIB_NAME}  ${LIB_SOURCES} ${LIB_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    include/stm32f4
    include/
    PRIVATE
    src
)


target_link_libraries(${LIB_NAME}
    PUBLIC
        # Provided libraries
        FreeRTOS::Timers
        FreeRTOS::Heap::1

        # stm32
        FreeRTOS::ARM_CM4F

        # native
        $<${COMPILE_NATIVE}:FreeRTOS::POSIX>
        $<${COMPILE_NATIVE}:Threads::Threads>
        

        # Defined libraries
        SwarmUS::HiveMind::FreeRTOS

    PRIVATE
        # Provided libraries


        #stm32
        $<${COMPILE_STM32}:STM32::NoSys>

        $<${COMPILE_STM32}:HAL::STM32::F4::RCC>
        $<${COMPILE_STM32}:HAL::STM32::F4::GPIO>
        $<${COMPILE_STM32}:HAL::STM32::F4::CORTEX>
        $<${COMPILE_STM32}:CMSIS::STM32::F407VG>
)


# TMP
target_compile_options(${LIB_NAME} PUBLIC 
    $<${COMPILE_STM32}:-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard>
    
)
target_link_options(${LIB_NAME} PUBLIC 
    $<${COMPILE_STM32}:-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard>
)


if (NOT CMAKE_CROSSCOMPILING)
    add_subdirectory(tests)
endif()