set(NODE_EXECUTABLE_NAME "tf_publisher")

set(NODE_HEADERS
        TopicConfigs.h
    )

set(NODE_SOURCES
        TfPublisher.cpp
    )

add_executable(${NODE_EXECUTABLE_NAME} ${NODE_SOURCES})

# Make ROS messages/services available in headers
add_dependencies(${NODE_EXECUTABLE_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})


target_include_directories(${NODE_EXECUTABLE_NAME}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        PRIVATE
        ${catkin_INCLUDE_DIRS}
        )

target_link_libraries(${NODE_EXECUTABLE_NAME}
        PUBLIC
        ${catkin_LIBRARIES}
        )

set_target_properties(${LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)