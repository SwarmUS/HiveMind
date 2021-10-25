set(LIB_NAME "swarmus-hivemind-bsp-stm32")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP::Platform")

get_filename_component(BSP_PLATFORM_CMAKE_DIR ${CMAKE_CURRENT_LIST_FILE} DIRECTORY)

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

if (${COMPILE_STM32_H735ZG})
    add_custom_target(
            mass_erase
            COMMAND openocd -f ${OPENOCD_CFG_PATH} -c 'init' -c 'halt' -c 'stm32h7x mass_erase 0' -c 'reset run' -c 'shutdown'
    )
endif()

add_subdirectory(hal)
add_subdirectory(src/interloc)

if (ENABLE_HARDWARE_TESTS)
    add_subdirectory(tests)
endif()


# Removing clang-tidy for this lib
if (DISABLE_EXTERNAL_WARNINGS) 
    set_target_properties(${LIB_NAME} PROPERTIES CXX_CLANG_TIDY "" )
    set_target_properties(${LIB_NAME} PROPERTIES C_CLANG_TIDY "" )
endif()

