set(LIB_NAME "swarmus-hivemind-freertos-stm32")
set(LIB_CONFIG_NAME "swarmus-hivemind-freertos-stm32-config")
set(LIB_ALIAS "SwarmUS::HiveMind::FreeRTOS::Platform")
set(LIB_CONFIG_ALIAS "SwarmUS::HiveMind::FreeRTOS::Platform::Config")

find_package(FreeRTOS COMPONENTS ARM_CM7 REQUIRED)

set(LIB_SOURCES
    platform_freertos_hooks.c
)

add_library(${LIB_NAME} INTERFACE)
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})
target_sources(${LIB_NAME} INTERFACE ${LIB_SOURCES})

target_link_libraries(${LIB_NAME}
    INTERFACE
        FreeRTOS::ARM_CM7
)

add_library(${LIB_CONFIG_NAME} INTERFACE)
add_library(${LIB_CONFIG_ALIAS} ALIAS ${LIB_CONFIG_NAME})
target_include_directories(${LIB_CONFIG_NAME}
    INTERFACE
        include
        ${FREERTOS_KERNEL_PATH}/portable/GCC/ARM_CM7/r0p1
)