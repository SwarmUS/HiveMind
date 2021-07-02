set(LIB_NAME "swarmus-hivemind-bsp-stm32-tests")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP::Platform::Tests")

set(LIB_SOURCES
        src/TestUI.cpp
        src/TestChannels.cpp
        src/TestUSB.cpp
)

set(LIB_PRIVATE_HEADERS
        include/IHardwareTest.h
        include/TestUI.h
        include/TestChannels.h
        include/TestUSB.h
)

add_library(${LIB_NAME}
    ${LIB_SOURCES}
    ${LIB_PRIVATE_HEADERS}
)

add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME} 
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        include
)

target_link_libraries(${LIB_NAME}
    PUBLIC
        # Defined libraries
        SwarmUS::HiveMind::OS
        SwarmUS::HiveMind::Logger

    PRIVATE
        SwarmUS::HiveMind::HAL
        SwarmUS::HiveMind::LWIP
)

# Removing clang-tidy for this lib
if (DISABLE_EXTERNAL_WARNINGS) 
    set_target_properties(${LIB_NAME} PROPERTIES CXX_CLANG_TIDY "" )
    set_target_properties(${LIB_NAME} PROPERTIES C_CLANG_TIDY "" )
endif()

