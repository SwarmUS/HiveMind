set(LIB_NAME swarmus-hivemind-hal-decawave-driver)
set(LIB_ALIAS "SwarmUS::HiveMind::HAL::Decawave::Driver")

set(LIB_SOURCES
        src/deca_device.c
        src/deca_params_init.c
    )

set(LIB_HEADERS
        include/deca_device_api.h
        include/deca_param_types.h
        include/deca_regs.h
        include/deca_types.h
        include/deca_version.h
    )


add_library(${LIB_NAME} INTERFACE)
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_sources(${LIB_NAME} INTERFACE ${LIB_SOURCES} ${LIB_HEADERS})

target_include_directories(${LIB_NAME}
    INTERFACE
        include
    )