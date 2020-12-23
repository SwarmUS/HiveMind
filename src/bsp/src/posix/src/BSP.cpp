#include "BSP.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

BSP::BSP() = default;
BSP::~BSP() = default;

void BSP::initChip(int argc, char** argv) {
    ros::init(argc, argv, "hive_mind");

    m_rosNodeHandle = new ros::NodeHandle("~");

    ros::Publisher chatter_pub = m_rosNodeHandle->advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);
    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    //    int count = 0;
    //    while (ros::ok()) {
    //        /**
    //         * This is a message object. You stuff it with data, and then publish it.
    //         */
    //        std_msgs::String msg;
    //
    //        std::stringstream ss;
    //        ss << "hello world " << count;
    //        msg.data = ss.str();
    //
    //        ROS_INFO("%s", msg.data.c_str());
    //
    //        /**
    //         * The publish() function is how you send messages. The parameter
    //         * is the message object. The type of this object must agree with the type
    //         * given as a template parameter to the advertise<>() call, as was done
    //         * in the constructor above.
    //         */
    //        chatter_pub.publish(msg);
    //        //
    //        ros::spinOnce();
    //
    //        loop_rate.sleep();
    //        ++count;
    //    }
}
