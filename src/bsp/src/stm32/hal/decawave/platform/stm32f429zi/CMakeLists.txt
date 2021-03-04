set(LIB_NAME swarmus-hivemind-hal-decawave-platform)
set(LIB_ALIAS "SwarmUS::HiveMind::HAL::Decawave::Platform")

set(LIB_SOURCES
        src/deca_mutex.c
        src/deca_sleep.c
        src/deca_spi.c
        src/port.c
    )

set(LIB_HEADERS
        include/port.h
        include/sleep.h
    )


add_library(${LIB_NAME} INTERFACE)
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_sources(${LIB_NAME} INTERFACE ${LIB_SOURCES} ${LIB_HEADERS})

target_include_directories(${LIB_NAME}
    INTERFACE
        include
    )

target_link_libraries(${LIB_NAME} INTERFACE SwarmUS::HiveMind::OS)