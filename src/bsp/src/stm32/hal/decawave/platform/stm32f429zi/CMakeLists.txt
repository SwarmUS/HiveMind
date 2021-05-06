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


add_library(${LIB_NAME} INTERFACE ${LIB_SOURCES} ${LIB_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

#target_sources(${LIB_NAME} INTERFACE )

target_include_directories(${LIB_NAME}
    PUBLIC
        include
    )

target_link_libraries(${LIB_NAME} INTERFACE SwarmUS::HiveMind::OS::FreeRTOS)