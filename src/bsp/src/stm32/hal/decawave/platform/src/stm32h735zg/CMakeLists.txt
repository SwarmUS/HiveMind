set(LIB_NAME swarmus-hivemind-hal-decawave-platform-device)
set(LIB_ALIAS "SwarmUS::HiveMind::HAL::Decawave::Platform::Device")

set(LIB_SOURCES
        deca_platform.c
        deca_platform.h
    )


add_library(${LIB_NAME} INTERFACE)
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_sources(${LIB_NAME} INTERFACE ${LIB_SOURCES})

target_include_directories(${LIB_NAME}
        INTERFACE
        ./
        )