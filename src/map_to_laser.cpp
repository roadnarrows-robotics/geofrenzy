#include <ros/ros.h>
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GetMap.h"
#include <tf/tf.h>
#include "nav_msgs/OccupancyGrid.h"
#include <sensor_msgs/LaserScan.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include "occupancy_grid_utils/exceptions.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

/**
 * This node subscribes to a topic of "dwell" with a message type of gf_entitlement and
 * publishes a topic of "Blink" with a Ros ColorRGBA message set according to the entitlement string
 * @todo modify program to recognize the the specific color entitlement when it is available in the string
 */


class MapToLaser
{
    public:
        MapToLaser()
        {

            pub_ = n_.advertise<sensor_msgs::LaserScan>("/gfscan", 1);
//            sub_ = n_.subscribe("/map", 1, &MapToLaser::callback, this);
           // sub_ = n_.subscribe("/mapviz/clicked_point", 1, &MapToLaser::callback, this);
//               sub_ = n_.subscribe("/initialpose", 1, &MapToLaser::callback, this);
               sub_ = n_.subscribe("/mavros/local_position/pose", 1, &MapToLaser::callback, this);
        }

//        void callback(const geometry_msgs::PoseWithCovarianceStamped& input)
     void callback(const geometry_msgs::PoseStamped& input)
        {

            ros::ServiceClient client = n_.serviceClient<nav_msgs::GetMap>("static_map");

            sensor_msgs::LaserScan laser;
            
              nav_msgs::GetMap srv;
              client.call(srv);
            
           // laser.angle_min=-3.14;
           // laser.angle_max=3.14;
	   unsigned int num_readings = 100;
           double laser_frequency = 40;
           laser.angle_min=-1;
           laser.angle_max=1;
            laser.angle_increment= 3.14/num_readings;
	    laser.time_increment = (1/laser_frequency) / (num_readings);
            laser.range_max=100; 
//            laser.scan_time=0.3333;
            printf("Laser\n");
	    //cout << input;
	    //cout.flush();
	geometry_msgs::Pose newPose;
	newPose.position.x = input.pose.position.x;
	newPose.position.y = input.pose.position.y;
	newPose.position.z = input.pose.position.z;
	newPose.orientation.x = input.pose.orientation.y;
	newPose.orientation.y = input.pose.orientation.x;
	newPose.orientation.z = -input.pose.orientation.z;
	newPose.orientation.w = input.pose.orientation.w;
                //      sensor_msgs::LaserScan::Ptr scan = occupancy_grid_utils::simulateRangeScan(srv.response.map,input.pose.pose,laser,true);
                      sensor_msgs::LaserScan::Ptr scan = occupancy_grid_utils::simulateRangeScan(srv.response.map,input.pose,laser,true);
                  //              sensor_msgs::LaserScan::Ptr scan = occupancy_grid_utils::simulateRangeScan(srv.response.map,input,laser,false);
            scan->header.frame_id = "base_laser";
            scan->header.stamp= ros::Time::now();
            pub_.publish(scan);
        }

    private:
        ros::NodeHandle n_;
        ros::Publisher pub_;
        ros::Subscriber sub_;

};//End of class MapToLaser

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "Map_to_Laser");

    //Create an object of class MapToLaser that will take care of everything
    MapToLaser SAPObject;

    ros::spin();

    return 0;
}