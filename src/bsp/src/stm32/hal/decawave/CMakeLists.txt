set(LIB_NAME swarmus-hivemind-hal-decawave)
set(LIB_ALIAS "SwarmUS::HiveMind::HAL::Decawave")

add_subdirectory(driver)
add_subdirectory(platform)

add_library(${LIB_NAME} STATIC)
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_link_libraries(${LIB_NAME}
    PUBLIC
        SwarmUS::HiveMind::HAL::Decawave::Driver
        SwarmUS::HiveMind::HAL::Decawave::Platform
    PRIVATE
        SwarmUS::HiveMind::HAL::STM32F4
)


