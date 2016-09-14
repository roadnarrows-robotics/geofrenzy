

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include <sstream>

/**
 * This node generates a pseudo GPS signal for use in testing the geofrenzy dns fence queries.
 * Currently it justs advertises a static location.
 * @todo Make the location dynamic
 * \param latitude Latitude of the fake signal, defaults to 36.160979
 * \param longitude Longitude of the fake signal, defaults to -115,139648
 */
int main(int argc, char **argv)
{
	double_t latitude,longitude;


  ros::init(argc, argv, "gf_fake_gps");
  ros::NodeHandle n;
  ros::Publisher color_pub = n.advertise<sensor_msgs::NavSatFix>("fix", 1000);
 

  ros::Rate loop_rate(2);
	n.param("latitide",latitude,36.160979);
	n.param("longitude",latitude,-115.139648);

  float count = 0;
  while (ros::ok())
  {

    sensor_msgs::NavSatFix msg;
	msg.latitude=latitude; //+(count/1000);
	msg.longitude=longitude;
	msg.header.stamp= ros::Time::now();

    ROS_INFO("latitude=%lf,longitude=%lf", latitude,longitude);

    color_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;

}
  return 0;
}

