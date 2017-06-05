#include "ros/ros.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Pose.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"


/**
 * FeatureCollection callback. The geofrenzy node publishes a feature collection...
 * this callback extracts the polygons from that message and transforms it into
 * an occupancy grid message to be published to the /map topic
 */

ros::Publisher map_pub;
ros::Publisher map_metadata_pub;
const int default_map_width = 10;
const int default_map_height = 10;
const double default_map_resolution = 1.0;

void featureCollectionCallback(const geometry_msgs::Polygon polygon){ //TODO: make this polygon array message
    ROS_INFO("Feature Collection Callback");

    nav_msgs::MapMetaData map_metadata;
    nav_msgs::OccupancyGrid occupancy_grid;
    std::vector<signed char> grid;
    int map_width;
    int map_height;
    double map_resolution;

    //Fetch values from parameter server
    ros::NodeHandle n;
    n.param("map_width", map_width, default_map_width);
    n.param("map_height", map_height, default_map_height);
    n.param("map_resolution", map_resolution, default_map_resolution);

    //Initialize grid
    int grid_size = map_width * map_height;
    grid.assign(grid_size, 0);
    grid[0] = (int8_t)100;
    occupancy_grid.data = grid;

    //Get Robot transform
    tf::TransformListener tf_listener; 
    geometry_msgs::PoseStamped gf_map_pose;
    geometry_msgs::PoseStamped base_footprint_pose;
    gf_map_pose.header.frame_id="base_footprint";
    gf_map_pose.header.stamp = ros::Time();
    //Shift gf_map to center on origin of world map
    gf_map_pose.pose.position.x = -(map_width/2.0)*map_resolution;
    gf_map_pose.pose.position.y = -(map_height/2.0)*map_resolution;
    gf_map_pose.pose.position.z = 0.0;
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    gf_map_pose.pose.orientation.x = q.x();
    gf_map_pose.pose.orientation.y = q.y();
    gf_map_pose.pose.orientation.z = q.z();
    gf_map_pose.pose.orientation.w = q.w();
    try{
      tf_listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(3.0));
      tf_listener.transformPose("map", gf_map_pose, base_footprint_pose);
    }
    catch(tf::TransformException ex){
      ROS_ERROR("Received exception trying to transform point from map to base_footprint: %s", ex.what());
    }
    
    map_metadata.resolution = map_resolution;
    map_metadata.width = map_width;
    map_metadata.height = map_height;
    map_metadata.map_load_time = ros::Time::now();

    //TODO: This should be robot.pose.position.x - (map_width/2.0)*map_resolution
    //pose.position.x = 0.0; //-base_footprint_pose.position.x - (map_width/2.0)*map_resolution;
    //pose.position.y = 0.0; //-base_footprint_pose.position.y - (map_height/2.0)*map_resolution;
    //pose.position.z = 0.0;
    //pose.position.x = -base_footprint_pose.pose.position.x;
    //pose.position.y = -base_footprint_pose.pose.position.y;
    //pose.position.z = -base_footprint_pose.pose.position.z;
    //pose.orientation.x = -base_footprint_pose.pose.orientation.x;
    //pose.orientation.y = -base_footprint_pose.pose.orientation.y;
    //pose.orientation.z = -base_footprint_pose.pose.orientation.z;
    //pose.orientation.w = -base_footprint_pose.pose.orientation.w;

    ROS_INFO("Map Transform: %.2f %.2f %.2f", base_footprint_pose.pose.position.x, base_footprint_pose.pose.position.y, base_footprint_pose.pose.position.z);
    map_metadata.origin = base_footprint_pose.pose;

    occupancy_grid.info = map_metadata;

    map_pub.publish(occupancy_grid);
    map_metadata_pub.publish(map_metadata);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "test_map_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("poly", 5, featureCollectionCallback);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/gf_map", 1);
    map_metadata_pub = nh.advertise<nav_msgs::MapMetaData>("/gf_map_metadata", 1);
    ros::spin();
}
