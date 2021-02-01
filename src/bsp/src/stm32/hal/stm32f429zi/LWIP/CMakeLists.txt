set(LIB_NAME "swarmus-hivemind-lwip")
set(LIB_ALIAS "SwarmUS::HiveMind::LWIP")

find_package(CMSIS_RTOS COMPONENTS
        STM32F429ZI
        REQUIRED)

find_package(LWIP COMPONENTS
        STM32F429ZI
        REQUIRED)

add_library(${LIB_NAME} STATIC)

add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
    PUBLIC
        Target
    PRIVATE
        ../Core/Inc
    )

target_link_libraries(${LIB_NAME}
    PUBLIC
        CMSIS::STM32::F4::LWIP
        CMSIS::STM32::F4::LWIP::IPV4
        CMSIS::STM32::F4::RTOS_V2
        SwarmUS::HiveMind::FreeRTOS

    PRIVATE
        CMSIS::STM32::F429ZI
        HAL::STM32::F4
        HAL::STM32::F4::ETH
    )

if (ENABLE_TARGET_IPERF_SERVER)
    target_link_libraries(${LIB_NAME}
        PUBLIC
            CMSIS::STM32::F4::LWIP::IPERF
    )
endif()

# Removing warnings and compile options
if (DISABLE_EXTERNAL_WARNINGS)
    target_compile_options(${LIB_NAME} PRIVATE -w)
    set_target_properties(${LIB_NAME} PROPERTIES CXX_CLANG_TIDY "" )
    set_target_properties(${LIB_NAME} PROPERTIES C_CLANG_TIDY "" )
endif()