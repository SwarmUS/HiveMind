if(${CMAKE_CROSSCOMPILING})
    message("-- Compiling for target without need for catkin")
    message("-- Skipping catkin related build steps")
else()
    message("-- Building for catkin env")
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        tf2
        tf2_ros
        tf2_geometry_msgs
        gazebo_msgs
        )

    add_message_files(
        DIRECTORY ros/msg
        FILES
        ${ROS_MESSAGES_LIST}
    )

   #TODO: Uncomment if service_generation is needed
   # add_service_files(
   #     DIRECTORY ros/srv
   #     FILES
   #     ${ROS_SERVICES_LIST}
   # )

    generate_messages(
        DEPENDENCIES
        std_msgs
    )

    catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
endif()
