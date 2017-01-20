
#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher color_pub = n.advertise<std_msgs::ColorRGBA>("gf_led_color", 1000);

    ros::Rate loop_rate(10);

    float count = 0;
    while (ros::ok())
    {
        std_msgs::ColorRGBA msg;
        msg.r = count;
        msg.g = 0.0;
        msg.b = 0.0;
        msg.a = 0.99;

        color_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
    return 0;
}

