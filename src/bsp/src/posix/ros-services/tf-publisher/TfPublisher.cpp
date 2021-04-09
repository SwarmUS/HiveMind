#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "TopicConfigs.h"

static double g_refreshRateS = 5;

tf::Transform getUnitTransform() {
    tf::Transform transform;

    transform.setRotation(tf::Quaternion::getIdentity());

    return transform;
}

void tfCallback(const gazebo_msgs::ModelStates& msg) {
    static tf::TransformBroadcaster s_tfBroadcaster;

    for (const auto& entityName : msg.name) {
        std::string odometryTopic(entityName + ODOMETRY_TOPIC_SUFFIX);

        // Send transform relative to "world"
        s_tfBroadcaster.sendTransform(
            tf::StampedTransform(getUnitTransform(), ros::Time::now(), "world", odometryTopic));

        // Sleep until for 1 ns
        ros::Duration(0, 1).sleep();
    }

    // Sleep until next interloc update
    ros::Duration(g_refreshRateS).sleep();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_publisher");

    ros::NodeHandle node("~");
    g_refreshRateS = node.param("refresh_rate_s", 0.1);

    ROS_INFO("Starting interloc service with refresh rate of %3.1f Hz", 1/g_refreshRateS);

    ros::Subscriber sub = node.subscribe(GAZEBO_MODEL_STATES_TOPIC, 1, &tfCallback);

    ros::spin();

    return 0;
}