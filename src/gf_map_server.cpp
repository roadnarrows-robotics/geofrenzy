//
// Add-on SDKs
//
#include "boost/assign.hpp"

//
// ROS
//
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"

//
// ROS generated Geofrenzy messages
//
#include "geofrenzy/GfDistFeatureCollection.h"
#include "geofrenzy/GfDistFeature.h"
#include "geofrenzy/Polygon64.h"

//
// ROS generated core messages
//
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GetMap.h"

//
// Geofrenzy
//
#include "gf_ros.h"
using namespace boost::assign;
using namespace geofrenzy::gf_ros;

namespace geofrenzy
{
    /*!
     * This class holds the data used to create the occupancy map
     */
    class MapGrid
    {
      public:
        int m_gridWidth;
        int m_gridLength;
        int m_gridSize;
        double m_gridResolution;
        std::vector<signed char> m_grid;

        MapGrid(int width, int length, double resolution);
    };

    MapGrid::MapGrid(int width, int length, double resolution)
        /*!
         * \brief Constructor for the MapGrid
         * \param width width in meters of the occupancy map
         * \param length length in meters of the occupancy map
         * \param resolution resolution in meters per pixel of occupancuy map
         */
    {
        m_gridWidth = (int)(width);
        m_gridLength = (int)(length);
        m_gridResolution = resolution;
        m_gridSize = m_gridWidth * m_gridLength;
        m_grid.assign(m_gridSize, 0);
    }

    class MapServer
    {
        /*!
        * This class holds the map server that does most of the work to
        * convert the Geofrenzy FDN data to ROS messages and services
        * to be easily used by ROS nodes
        */
      public:
        void featureCollectionCallback(const geofrenzy::GfDistFeatureCollection distFeatures);
        void advertisePublishers(int nQueueDepth=1);
        void subscribeToTopics(int nQueueDepth=5);
        void advertiseServices();
        void publishMap();
        bool getMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);

        MapServer(ros::NodeHandle &nh, double hz, uint64_t gf_class_idx);

      private:
        ros::NodeHandle &m_nh;            ///< node handle
        double          m_hz;             ///< cycle hertz
        uint64_t        m_fenceClass;    ///< geofrenzy class index

        // ROS services, publishers, subscriptions.
        MapServices       m_services;       ///< Geofrenzy map server services
        MapClientServices m_clientServices; ///< Geofrenzy map server client services
        MapPublishers     m_publishers;     ///< Geofrenzy map server publishers
        MapSubscriptions  m_subscriptions;  ///< Geofrenzy map server subscriptions

        //ROS Transform Listener
        tf::TransformListener m_tfListener;
        std::string m_globalFrame;
        std::string m_robotFrame;

        // Messaging processing overhead
        ros::Time m_atime;        ///< last callback time
        int       m_nPublishCnt;  ///< publish counter

        //Messages for publishing
        nav_msgs::OccupancyGrid m_occupancyGrid;
        nav_msgs::MapMetaData m_mapMetadata;

        void drawLine(double x1, double y1, double x2, double y2, MapGrid &grid);
        void transformPoint(geometry_msgs::Point pIn, geometry_msgs::Point &pOut, double dX, double dY, double dZ);
        bool isWithinMapBounds(geometry_msgs::Point p1, geometry_msgs::Point p2, MapGrid grid);
        bool checkIntersection(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point p3, geometry_msgs::Point p4);
    };


    MapServer::MapServer(ros::NodeHandle &nh, double hz, uint64_t gf_class_idx) :
        m_fenceClass(gf_class_idx), m_nh(nh), m_hz(hz), m_atime(0.0), m_nPublishCnt(0)
    {
        /*!
         * \brief Constructor for MapServer
         */
        ROS_DEBUG_STREAM("MapServer gf_class_idx = " << m_fenceClass);
        //global frame and robot frame used for transformation
        m_nh.param(ParamNameGlobalFrame, m_globalFrame, GlobalFrameDft);
        m_nh.param(ParamNameRobotFrame, m_robotFrame, RobotFrameDft);
    }

    /*!
     * \brief Advertise all publishers.
     *
     * \param nQueueDepth   Maximum queue depth.
     */
    void MapServer::advertisePublishers(int nQueueDepth)
    {
      m_publishers[TopicNameMap] = m_nh.advertise<nav_msgs::OccupancyGrid>(TopicNameMap, nQueueDepth, true);
      m_publishers[TopicNameMapMD] = m_nh.advertise<nav_msgs::MapMetaData>(TopicNameMapMD, nQueueDepth, true);
    }

    /*!
     * \brief Subscribe to all topics.
     *
     * \param nQueueDepth   Maximum queue depth.
     */
    void MapServer::subscribeToTopics(int nQueueDepth)
    {
        // TODO: resolve subscription topic name
        //m_subscriptions[TopicNameFcDist] = m_nh.subscribe(TopicNameFcDist, nQueueDepth, &MapServer::featureCollectionCallback, &(*this));
        m_subscriptions["/gf_server_168/geofrenzy/featureCollection/distance"] = m_nh.subscribe("/gf_server_168/geofrenzy/featureCollection/distance", 5, &MapServer::featureCollectionCallback, &(*this));

    }

    /*!
     * \brief Advertise all map server services.
     *
     */
    void MapServer::advertiseServices()
    {
        m_services[ServiceNameGetMap] = m_nh.advertiseService(ServiceNameGetMap, &MapServer::getMap, &(*this));
    }

    void MapServer::transformPoint(geometry_msgs::Point pIn, geometry_msgs::Point &pOut, double dX, double dY, double dZ){
        pOut.x = pIn.x + dX;
        pOut.y = pIn.y + dY;
        pOut.z = pIn.z + dZ;
    }

    /*
     * Check if two lines intersect. Line1 defined by p1, p2; Line2 defined by p3, p4
     */
    bool MapServer::checkIntersection(geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point p3, geometry_msgs::Point p4){
        double s1_x, s1_y, s2_x, s2_y;
        s1_x = p2.x - p1.x;
        s1_y = p2.y - p1.y;
        s2_x = p4.x - p3.x;
        s2_y = p4.y - p3.y;

        double s, t;
        s = (-s1_y * (p1.x - p3.x) + s1_x * (p1.y - p3.y)) / (-s2_x * s1_y + s1_x * s2_y);
        t = ( s2_x * (p1.y - p3.y) - s2_y * (p1.x - p3.x)) / (-s2_x * s1_y + s1_x * s2_y);

        if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
        {
            // Collision detected
            return 1;
        }

        return 0; // No collision
    }

    bool MapServer::isWithinMapBounds(geometry_msgs::Point p1, geometry_msgs::Point p2, MapGrid grid){
        //Define points of occupancy grid corners
        geometry_msgs::Point pA;
        geometry_msgs::Point pB;
        geometry_msgs::Point pC;
        geometry_msgs::Point pD;
        pA.x = 0;
        pA.y = 0;
        pB.x = grid.m_gridWidth*grid.m_gridResolution;
        pB.y = 0;
        pC.x = grid.m_gridWidth*grid.m_gridResolution;
        pC.y = grid.m_gridLength*grid.m_gridResolution;
        pD.x = 0;
        pD.y = grid.m_gridLength*grid.m_gridResolution;

        //Check if either point is within rectangle bounds
        if(p1.x <= pC.x && p1.x >= pA.x && p1.y <= pC.y && p1.y >= pA.y)
            return true;
        if(p2.x <= pC.x && p2.x >= pA.x && p2.y <= pC.y && p2.y >= pA.y)
            return true;
        //Check if line intersects any of the rectangle edges
        if(checkIntersection(p1, p2, pA, pB))
            return true;
        if(checkIntersection(p1, p2, pB, pC))
            return true;
        if(checkIntersection(p1, p2, pC, pD))
            return true;
        if(checkIntersection(p1, p2, pD, pA))
            return true;

        //Outside of grid bounds
        return false;
    }

    void MapServer::drawLine(double x1, double y1, double x2, double y2, MapGrid &grid)
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
        x1/=grid.m_gridResolution;
        y1/=grid.m_gridResolution;
        x2/=grid.m_gridResolution;
        y2/=grid.m_gridResolution;

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

        int maxX;
        if(steep){
            maxX = std::min((int)x2, (int)grid.m_gridLength);
        }else{
            maxX = std::min((int)x2, (int)grid.m_gridWidth);
        }

        for (int x = (int)x1; x < maxX; x++)
        {
            if (steep)
            {
                if(x >= 0 && y >= 0 && y<grid.m_gridWidth && x<grid.m_gridLength){
                    grid.m_grid[(x * (int)grid.m_gridWidth) + (int)y] = (int8_t)100;
                }
            }
            else
            {
                if(x >= 0 && y>=0 && y<grid.m_gridLength && x<grid.m_gridWidth){
                    grid.m_grid[(y * (int)grid.m_gridWidth) + (int)x] = (int8_t)100;
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


    /*!
     * \brief Publish.
     *
     * Call in main loop.
     */
    void MapServer::publishMap(){
        if(m_nPublishCnt > 0){
            m_publishers[TopicNameMap].publish(m_occupancyGrid);
            m_publishers[TopicNameMapMD].publish(m_mapMetadata);
            --m_nPublishCnt;
        }
    }

    /**
     * FeatureCollection callback. The geofrenzy node publishes a feature collection,
     * this callback extracts the polygons from that message and transforms it into
     * an occupancy grid message to be published to the /map topic
     */
    void MapServer::featureCollectionCallback(const geofrenzy::GfDistFeatureCollection distFeatures){
        int mapWidth;
        int mapHeight;
        double mapResolution;

        ROS_DEBUG("Feature Collection Callback");

        //Fetch values from parameter server
        m_nh.param(ParamNameMapWidth, mapWidth, MapWidthDft);
        m_nh.param(ParamNameMapHeight, mapHeight, MapHeightDft);
        m_nh.param(ParamNameMapResolution, mapResolution, MapResolutionDft);

        //Get Robot transform
        m_atime = ros::Time::now();
        geometry_msgs::PoseStamped gfMapPose;
        gfMapPose.header.frame_id=m_robotFrame;
        gfMapPose.header.stamp = m_atime;

        tf::StampedTransform transform;
        try{
          m_tfListener.waitForTransform(m_globalFrame, m_robotFrame, m_atime, ros::Duration(3.0));
          m_tfListener.lookupTransform(m_globalFrame, m_robotFrame, m_atime, transform);
        }
        catch(tf::TransformException ex){
          ROS_ERROR("Received exception trying to transform point from map to base_footprint: %s", ex.what());
        }

        //Initialize grid
        MapGrid mapGrid(mapWidth, mapHeight, mapResolution);

        /*Iterate through GfDistFeatureCollection
         * Get all geometries and iterate through points, drawing edges on occupancy grid
         */
        double dx = (mapGrid.m_gridWidth/2.0)*mapGrid.m_gridResolution;
        double dy = (mapGrid.m_gridLength/2.0)*mapGrid.m_gridResolution;
        geofrenzy::GfDistFeatureCollection featureCollection = distFeatures;
        for(std::vector<geofrenzy::GfDistFeature>::iterator feature_it = featureCollection.features.begin(); feature_it != featureCollection.features.end(); feature_it++){
            geofrenzy::GfDistFeature feature = *(feature_it);
            for(std::vector<geofrenzy::Polygon64>::iterator geometry_it = feature.geometry.begin(); geometry_it != feature.geometry.end(); ++geometry_it){
                geofrenzy::Polygon64 geometry = *(geometry_it);
                for(std::vector<geometry_msgs::Point>::iterator point_it = geometry.points.begin(); point_it != geometry.points.end(); ++point_it){
                    geometry_msgs::Point p1 = *point_it;
                    geometry_msgs::Point p2;
                    if((point_it+1) != geometry.points.end()){
                        p2 = *(point_it+1);
                    }else{
                        p2 = *(geometry.points.begin());
                    }
                    transformPoint(p1, p1, dx, dy, 0.0);
                    transformPoint(p2, p2, dx, dy, 0.0);
                    if(isWithinMapBounds(p1, p2, mapGrid)){
                        drawLine(p1.x, p1.y, p2.x, p2.y, mapGrid);
                    }
                }
            }
        }

        m_occupancyGrid.data = mapGrid.m_grid;

        gfMapPose.pose.position.x = -dx + transform.getOrigin().getX();
        gfMapPose.pose.position.y = -dy + transform.getOrigin().getY();
        gfMapPose.pose.position.z = 0.0;
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        gfMapPose.pose.orientation.x = q.x();
        gfMapPose.pose.orientation.y = q.y();
        gfMapPose.pose.orientation.z = q.z();
        gfMapPose.pose.orientation.w = q.w();

        m_mapMetadata.resolution = mapGrid.m_gridResolution;
        m_mapMetadata.width = mapGrid.m_gridWidth;
        m_mapMetadata.height = mapGrid.m_gridLength;
        m_mapMetadata.map_load_time = m_atime;
        m_mapMetadata.origin = gfMapPose.pose;
        m_occupancyGrid.info = m_mapMetadata;
        m_occupancyGrid.header.frame_id = m_globalFrame;
        m_occupancyGrid.header.stamp = m_atime;
        ++m_nPublishCnt;

    }

    /*!
     * \brief GetMap Service
     *
     * \param req   unused
     * \param res   Map Response
     */
    bool MapServer::getMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
    {
        ROS_INFO("Requesting Map");
        // request is empty; we ignore it

        nav_msgs::GetMap::Response mapResponse;
        mapResponse.map = m_occupancyGrid;
        res = mapResponse;
        ROS_INFO("Sending map");
        return true;
    }
} //namespace geofrenzy

int main(int argc, char **argv){

    double  hz;

    // get command-line Geofrenzy class index
    uint64_t gfClassIdx = paramClassIndex(argc, argv);

    // make a unique node name from the command line class index argument
    std::string nodeName = makeNodeName(NodeRootMapServer, gfClassIdx);

    ROS_INFO_STREAM("Starting node with name: " << nodeName);
    //
    // Initialize the node. Parse the command line arguments and environment to
    // determine ROS options such as node name, namespace and remappings.
    // This call does not contact the master. This lets you use
    // ros::master::check() and other ROS functions after calling ros::init()
    // to check on the status of the master.
    ros::init(argc, argv, nodeName);

    // actual ROS-given node name
    nodeName = ros::this_node::getName();

    //
    // A ctrl-c interrupt will stop attempts to connect to the ROS core.
    //
    ros::NodeHandle nh(nodeName);

    //
    // Parse node command-line private (and static) arguments.
    //
    nh.param("hz", hz, 10.0);   // node hertz rate

    //
    // Failed to connect.
    //
    if( !ros::master::check() )
    {
      // add optional non-ROS unit tests here, then simply exit.
      ROS_INFO_STREAM(nodeName << ": ROS master not running.");
      return 0;
    }

    ROS_INFO_STREAM(nodeName << ": ROS master running.");

    //
    //Create map server node
    geofrenzy::MapServer mapServer(nh, hz, gfClassIdx);

    //
    // Advertise publishers.
    //
    mapServer.advertisePublishers();

    ROS_INFO_STREAM(nodeName << ": Advertised publishers registered.");

    //
    // Subscribed to topics.
    //
    mapServer.subscribeToTopics();

    ROS_INFO_STREAM(nodeName << ": Subscribed topics registered.");

    //
    // Advertise services.
    //
    mapServer.advertiseServices();

    ROS_INFO_STREAM(nodeName << ": Advertised services registered.");

    // set loop rate in Hertz
    ros::Rate loop_rate(hz);

    ROS_INFO_STREAM(nodeName << ": Ready.");

    //
    // Main loop.
    //
    while( ros::ok() )
    {
      // make any callbacks on pending ROS events
      ros::spinOnce();

      // publish all readied advertised topics
      mapServer.publishMap();

      // sleep to keep at loop rate
      loop_rate.sleep();
    }

    return 0;
}
