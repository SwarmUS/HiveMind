set(LIB_NAME "swarmus-hivemind-bsp-posix")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP::Platform")

set(LIB_SOURCES
    src/BSP.cpp
    src/BSPContainer.cpp

    src/UserInterface.cpp

    src/TCPClient.cpp
    src/SocketContainer.cpp
    src/SocketFactory.cpp
    src/SettingsContainer.cpp
    src/TCPServer.cpp)

set(LIB_PRIVATE_HEADERS
    include/BSP.h
    include/UserInterface.h
    include/SocketFactory.h
    include/TCPClient.h
    include/TCPServer.h
    include/USBMock.h
    include/InterlocManager.h)

add_library(${LIB_NAME} STATIC ${LIB_SOURCES} ${LIB_PRIVATE_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

# Make ROS messages/services available in headers
add_dependencies(${LIB_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_include_directories(${LIB_NAME} SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
target_include_directories(${LIB_NAME}
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        ${BSP_CMAKE_DIR}/include
        include
)

target_link_libraries(${LIB_NAME}
    PUBLIC
        # Defined libraries
        SwarmUS::HiveMind::OS
        SwarmUS::HiveMind::Logger
        ${catkin_LIBRARIES}
    PRIVATE
        SwarmUS::Propolis::Pheromones::Common
)

# Mock linker library
add_library(posix-linker INTERFACE)
add_library(${LIB_BSP_LINKER_ALIAS} ALIAS posix-linker)

add_custom_target(
    run
    COMMAND $<TARGET_FILE:${EXECUTABLE_NAME}>
)
