set(LIB_NAME "swarmus-hivemind-bsp-stm32")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP::Platform")

set(LIB_SOURCES
    src/BSP.cpp
    src/BSPContainer.cpp
    src/UserInterface.cpp
    src/SocketContainer.cpp
    src/HostUart.cpp
    src/CRC.cpp
    src/SocketFactory.cpp
    src/TCPClient.cpp
)

set(LIB_PRIVATE_HEADERS
    include/BSP.h
    include/UserInterface.h
    include/HostUart.h
    include/CRC.h
    include/SocketFactory.h
    include/TCPClient.h
)


add_library(${LIB_NAME}
    ${LIB_SOURCES}
    ${LIB_PRIVATE_HEADERS}
)

add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME} 
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        ${BSP_CMAKE_DIR}/include
        include
)

target_link_libraries(${LIB_NAME}
    PUBLIC
        # Defined libraries
        SwarmUS::HiveMind::FreeRTOS
        SwarmUS::HiveMind::Logger

    PRIVATE
        SwarmUS::HiveMind::HAL
        SwarmUS::HiveMind::LWIP
)

add_custom_target(
    flash
    COMMAND openocd  -f ${OPENOCD_CFG_PATH} -c 'program $<TARGET_FILE:${EXECUTABLE_NAME}> verify reset exit'
)

add_subdirectory(hal)
