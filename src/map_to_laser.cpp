#include <ros/ros.h>
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GetMap.h"
#include <tf/tf.h>
#include "nav_msgs/OccupancyGrid.h"
#include <sensor_msgs/LaserScan.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include "occupancy_grid_utils/exceptions.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

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
               sub_ = n_.subscribe("/initialpose", 1, &MapToLaser::callback, this);
        }

        void callback(const geometry_msgs::PoseWithCovarianceStamped& input)
    // void callback(const geometry_msgs::PointStamped& input)
        {

            ros::ServiceClient client = n_.serviceClient<nav_msgs::GetMap>("static_map");

            sensor_msgs::LaserScan laser;


            
              nav_msgs::GetMap srv;
              client.call(srv);
              

            
            
           // laser.angle_min=-3.14;
           // laser.angle_max=3.14;
           laser.angle_min=-1;
           laser.angle_max=1;
            laser.angle_increment= .01;
            laser.range_max=100; 
            printf("Laser\n");
                      sensor_msgs::LaserScan::Ptr scan = occupancy_grid_utils::simulateRangeScan(srv.response.map,input.pose.pose,laser,true);
                  //              sensor_msgs::LaserScan::Ptr scan = occupancy_grid_utils::simulateRangeScan(srv.response.map,input,laser,false);
            scan->header.frame_id = "map";
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
