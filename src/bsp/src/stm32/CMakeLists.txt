set(LIB_NAME "swarmus-hivemind-bsp-stm32")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP::Platform")

set(LIB_SOURCES
    src/BSP.cpp
    src/BSPContainer.cpp
    src/UserInterface.cpp
    src/HardwareCRC.cpp
    src/SocketFactory.cpp
    src/TCPClient.cpp
    src/SettingsContainer.cpp
    src/SpiEsp.cpp
    src/USB.cpp
    src/Math.cpp
    src/PersistantStorageManager.cpp
)

set(LIB_PRIVATE_HEADERS
    include/BSP.h
    include/UserInterface.h
    include/HardwareCRC.h
    include/SocketFactory.h
    include/TCPClient.h
    include/DefaultSettings.h
    include/SpiEsp.h
    include/SpiHeader.h
    include/USB.h
    include/PersistantStorage.h
    include/PersistantStorageManager.h
)

include(DefaultSettings.cmake)
configure_file(include/DefaultSettings.in.h include/DefaultSettings.h)

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

        #Include the generated DefaultSettings.h
        ${CMAKE_CURRENT_BINARY_DIR}/include
)

target_link_libraries(${LIB_NAME}
    PUBLIC
        # Defined libraries
        SwarmUS::HiveMind::OS
        SwarmUS::HiveMind::Logger

    PRIVATE
        SwarmUS::HiveMind::HAL
        SwarmUS::HiveMind::LWIP
        SwarmUS::Propolis::Pheromones
        SwarmUS::Propolis::Cpp::Common
        SwarmUS::HiveMind::BSP::Platform::Interloc
)

add_custom_target(
    flash
    COMMAND openocd  -f ${OPENOCD_CFG_PATH} -c 'program $<TARGET_FILE:${EXECUTABLE_NAME}> verify reset exit'
)

add_subdirectory(hal)
add_subdirectory(src/interloc)

