set(LIB_NAME "swarmus-hivemind-freertos-posix")
set(LIB_ALIAS "SwarmUS::HiveMind::FreeRTOS::Platform")

add_library(${LIB_NAME} ${LIB_SOURCES} ${LIB_HEADERS} INTERFACE)
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

find_package(FreeRTOS REQUIRED)

target_link_libraries(${LIB_NAME}
    INTERFACE
        FreeRTOS::POSIX
)