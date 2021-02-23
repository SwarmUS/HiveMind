set(LIB_NAME swarmus-hivemind-hal-decawave-platform)
set(LIB_ALIAS "SwarmUS::HiveMind::HAL::Decawave::Platform")

set(LIB_SOURCES
        src/deca_mutex.c
        src/deca_range_tables.c
        src/deca_sleep.c
        src/deca_spi.c
        src/port.c
        src/stdio.c
    )

set(LIB_HEADERS
        include/deca_spi.h
        include/port.h
        include/sleep.h
        include/stdio.h
    )


add_library(${LIB_NAME} INTERFACE)
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_sources(${LIB_NAME} INTERFACE ${LIB_SOURCES} ${LIB_HEADERS})

target_include_directories(${LIB_NAME}
    INTERFACE
        include
    )
