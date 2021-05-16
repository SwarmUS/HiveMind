set(LIB_NAME swarmus-hivemind-hal-decawave)
set(LIB_ALIAS "SwarmUS::HiveMind::HAL::Decawave")

add_subdirectory(driver)
add_subdirectory(platform)

add_library(${LIB_NAME} STATIC)
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_link_libraries(${LIB_NAME}
    PUBLIC
        SwarmUS::HiveMind::HAL::Decawave::Platform
    PRIVATE
        SwarmUS::HiveMind::HAL::Decawave::Driver
        SwarmUS::HiveMind::HAL::STM32
)

get_target_property(DECA_DRIVER_INCLUDE SwarmUS::HiveMind::HAL::Decawave::Driver INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(${LIB_NAME}
    SYSTEM PUBLIC 
        ${DECA_DRIVER_INCLUDE}
)


# Removing warnings and compile options
if (DISABLE_EXTERNAL_WARNINGS) 
    target_compile_options(${LIB_NAME} PRIVATE -w)
    set_target_properties(${LIB_NAME} PROPERTIES CXX_CLANG_TIDY "" )
    set_target_properties(${LIB_NAME} PROPERTIES C_CLANG_TIDY "" )
endif()
