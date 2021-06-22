set(LIB_NAME "swarmus-hivemind-freertos-posix")
set(LIB_CONFIG_NAME "swarmus-hivemind-freertos-posix-config")
set(LIB_ALIAS "SwarmUS::HiveMind::FreeRTOS::Platform")
set(LIB_CONFIG_ALIAS "SwarmUS::HiveMind::FreeRTOS::Platform::Config")

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(FreeRTOS COMPONENTS POSIX REQUIRED)

add_library(${LIB_NAME} INTERFACE)
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_link_libraries(${LIB_NAME}
    INTERFACE
        Threads::Threads
        FreeRTOS::POSIX
)

add_library(${LIB_CONFIG_NAME} INTERFACE)
add_library(${LIB_CONFIG_ALIAS} ALIAS ${LIB_CONFIG_NAME})
target_include_directories(${LIB_CONFIG_NAME}
    INTERFACE
        include
        ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/Posix
)