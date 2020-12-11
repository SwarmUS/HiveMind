set(LIB_NAME "swarmus-hivemind-freertos-stm32")
set(LIB_ALIAS "SwarmUS::HiveMind::FreeRTOS::Platform")

add_library(${LIB_NAME} ${LIB_SOURCES} ${LIB_HEADERS} INTERFACE)
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

find_package(FreeRTOS COMPONENTS
            ARM_CM4F
            REQUIRED)

target_link_libraries(${LIB_NAME}
    INTERFACE
        FreeRTOS::ARM_CM4F
)

target_include_directories(${LIB_NAME}
    INTERFACE
        include
)

            
