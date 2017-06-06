#include "ros/ros.h"
#include "geofrenzy/GfDistFeatureCollection.h"
#include "geofrenzy/GfDistFeature.h"
#include "geofrenzy/Polygon64.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
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
const int default_map_width = 100;
const int default_map_height = 100;
const double default_map_resolution = 0.1;

class MapGrid
/**
 * This class holds the data used to create the occupancy map
 */

{
  public:
    int gridwidth;
    int gridlength;
    int gridsize;
    double grid_resolution;
    std::vector<signed char> grid;

    MapGrid(int width, int length, double resolution);
};

MapGrid::MapGrid(int width, int length, double resolution)
/**
 * This is the constructor for the MapGrid
 * \param width width in meters of the occupancy map
 * \param length length in meters of the occupancy map
 */
{
    gridwidth = (int)(width);
    gridlength = (int)(length);
    grid_resolution = resolution;
    gridsize = gridwidth * gridlength;
    grid.assign(gridsize, 0);
}

void transformPoint(geometry_msgs::Point p_in, geometry_msgs::Point &p_out, double dX, double dY, double dZ){
    p_out.x = p_in.x + dX;
    p_out.y = p_in.y + dY;
    p_out.z = p_in.z + dZ;
}

void Line(double x1, double y1, double x2, double y2, MapGrid &grid)
{
    /**
    * This function uses Bresenham's line algorithm to "draw" the edges
    * in the grid between the points listed in the polygon definition
    * \param x1 x location of first point
    * \param y1 y location of first point
    * \param x2 x location of second point
    * \param y2 y location of the second point
    * \param grid - output grid to "draw" line in
    */
    //Scale distances based on map resolution
    x1/=grid.grid_resolution;
    y1/=grid.grid_resolution;
    x2/=grid.grid_resolution;
    y2/=grid.grid_resolution;

    // Bresenham's line algorithm
    const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
    if (steep)
    {
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    if (x1 > x2)
    {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    const double dx = x2 - x1;
    const double dy = fabs(y2 - y1);

    float error = dx / 2.0f;
    const int ystep = (y1 < y2) ? 1 : -1;
    int y = (int)y1;

    const int maxX = (int)x2;
    for (int x = (int)x1; x < maxX; x++)
    {
        if (steep)
        {
            if((0 <= x && x < grid.gridlength) && (0 <= y && y < grid.gridlength)){
                grid.grid[(x * (int)grid.gridwidth) + (int)y] = (int8_t)100;
                grid.grid[(x * (int)grid.gridwidth) + (int)y + 1] = (int8_t)100;
            }
        }
        else
        {
            if((0 <= y && y < grid.gridlength) && (0 <= x && x < grid.gridwidth)){
                grid.grid[(y * (int)grid.gridwidth) + (int)x] = (int8_t)100;
                grid.grid[(y * (int)grid.gridwidth) + (int)x + 1] = (int8_t)100;
            }
        }

        error -= dy;
        if (error < 0)
        {
            y += ystep;
            error += dx;
        }
    }
}

void featureCollectionCallback(const geofrenzy::GfDistFeatureCollection distFeatures){ //TODO: make this polygon array message
    //ROS_INFO("Feature Collection Callback");

    geofrenzy::Polygon64 polygon;
    geometry_msgs::PointStamped new_point;
    geometry_msgs::PointStamped trans_point;
    tf::TransformListener tf_listener;

    /*
     * Setup test static Polygon64 for testing robot-centric map
     */
    new_point.header.frame_id="map";
    new_point.header.stamp = ros::Time();
    new_point.point.x = -5.0;
    new_point.point.y = -5.0;
    new_point.point.z = 0.0;
    try{
      tf_listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(3.0));
      tf_listener.transformPoint("base_footprint", new_point, trans_point);
    }
    catch(tf::TransformException ex){
      ROS_ERROR("Received exception trying to transform point from map to base_footprint: %s", ex.what());
    }
    polygon.points.push_back(trans_point.point);

    new_point.point.x = 0.0;
    new_point.point.y = 5.0;
    new_point.point.z = 0.0;
    try{
      tf_listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(3.0));
      tf_listener.transformPoint("base_footprint", new_point, trans_point);
    }
    catch(tf::TransformException ex){
      ROS_ERROR("Received exception trying to transform point from map to base_footprint: %s", ex.what());
    }
    polygon.points.push_back(trans_point.point);

    new_point.point.x = 5.0;
    new_point.point.y = -5.0;
    new_point.point.z = 0.0;
    try{
      tf_listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(3.0));
      tf_listener.transformPoint("base_footprint", new_point, trans_point);
    }
    catch(tf::TransformException ex){
      ROS_ERROR("Received exception trying to transform point from map to base_footprint: %s", ex.what());
    }
    polygon.points.push_back(trans_point.point);

    /*
     * TODO: Use GfDistFeatureCollection and pull out all Polygon64 messages for this.
     */

    nav_msgs::MapMetaData map_metadata;
    nav_msgs::OccupancyGrid occupancy_grid;
    int map_width;
    int map_height;
    double map_resolution;

    //Fetch values from parameter server
    ros::NodeHandle n;
    n.param("map_width", map_width, default_map_width);
    n.param("map_height", map_height, default_map_height);
    n.param("map_resolution", map_resolution, default_map_resolution);

    //Initialize grid
    MapGrid map_grid(map_width, map_height, map_resolution);

    //Iterate through Polygons - eventually these will come from GfDistFeatureCollection message
    double dx = (map_width/2.0)*map_resolution;
    double dy = (map_height/2.0)*map_resolution;
    for (std::vector<geometry_msgs::Point>::iterator it = polygon.points.begin(); it != polygon.points.end(); ++it){
        geometry_msgs::Point p1 = *it;
        geometry_msgs::Point p2;
        if((it+1) != polygon.points.end()){
            p2 = *(it+1);
        }else{
            p2 = *(polygon.points.begin());
        }
        transformPoint(p1, p1, dx, dy, 0.0);
        transformPoint(p2, p2, dx, dy, 0.0);
        Line(p1.x, p1.y, p2.x, p2.y, map_grid);
    }
    occupancy_grid.data = map_grid.grid;

    //Get Robot transform
    geometry_msgs::PoseStamped gf_map_pose;
    geometry_msgs::PoseStamped base_footprint_pose;
    gf_map_pose.header.frame_id="base_footprint";
    gf_map_pose.header.stamp = ros::Time();
    //Shift gf_map to center on origin of world map
    gf_map_pose.pose.position.x = -dx;
    gf_map_pose.pose.position.y = -dy;
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
    map_metadata.origin = base_footprint_pose.pose;

    occupancy_grid.info = map_metadata;

    map_pub.publish(occupancy_grid);
    map_metadata_pub.publish(map_metadata);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "gf_map_server2");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("poly", 5, featureCollectionCallback);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/gf_map", 1);
    map_metadata_pub = nh.advertise<nav_msgs::MapMetaData>("/gf_map_metadata", 1);
    ros::spin();
}
