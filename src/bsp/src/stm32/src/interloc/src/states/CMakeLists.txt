set(LIB_NAME "swarmus-hivemind-bsp-stm32-interloc-states")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP::Platform::Interloc::States")

set(LIB_SOURCES
)

set(LIB_HEADERS
    include/states/IInterlocState.h
)

add_library(${LIB_NAME} INTERFACE)
target_sources(${LIB_NAME} INTERFACE ${LIB_SOURCES})
target_sources(${LIB_NAME} INTERFACE ${LIB_HEADERS})

add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
        INTERFACE
        include
        )

target_link_libraries(${LIB_NAME}
        INTERFACE
        # Defined libraries
        SwarmUS::HiveMind::OS
        SwarmUS::HiveMind::Logger
        SwarmUS::HiveMind::HAL
        )