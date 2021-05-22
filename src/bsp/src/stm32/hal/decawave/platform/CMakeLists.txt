set(LIB_NAME swarmus-hivemind-hal-decawave-platform)
set(LIB_ALIAS "SwarmUS::HiveMind::HAL::Decawave::Platform")

set(LIB_SOURCES
        src/deca_mutex.c
        src/deca_sleep.c
        src/deca_spi.c
        src/deca_port.c
    )

set(LIB_HEADERS
        include/deca_port.h
    )

add_library(${LIB_NAME} INTERFACE)
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_sources(${LIB_NAME} INTERFACE ${LIB_SOURCES} ${LIB_HEADERS})

target_include_directories(${LIB_NAME}
    INTERFACE
        include
    )

if (${COMPILE_STM32_F429ZI})
    add_subdirectory(src/stm32f429zi)
elseif (${COMPILE_STM32_H735ZG})
    add_subdirectory(src/stm32h735zg)
else()
    message(FATAL_ERROR "STM PORT: The current board is not supported" )
endif()

target_link_libraries(${LIB_NAME} INTERFACE
        SwarmUS::HiveMind::HAL::Decawave::Platform::Device
        SwarmUS::HiveMind::OS::FreeRTOS
        )