if(COMPILE_WITHOUT_CATKIN)
    message("-- Compiling for target without need for catkin")
    message("-- Skipping catkin related build steps")
else()
    message("-- Building for catkin env")
    find_package(catkin REQUIRED COMPONENTS
            roscpp
            rospy
            std_msgs
            message_generation
            )
    catkin_package()
endif()