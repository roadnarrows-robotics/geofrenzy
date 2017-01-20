// ----------------------------------------------------------------------------
// Copyright (C) 2002-2006 Marcin Kalicinski
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
// For more information, see www.boost.org
// ----------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>

#include <cmath>
#include <iostream>
#include <string>
using namespace std;
#include <ctime>
#include <cstdlib>
#include <algorithm>

#include <vector>

#include <set>
#include <exception>
#include <iostream>
#include <limits>
#include <iterator>

#include <string>
#include <fstream>
#include <streambuf>
#include "jsoncpp/json/json.h"
#include "jsoncpp/json/writer.h"
#include <iostream>

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"
#include <tf/tf.h>
#include "std_msgs/String.h"
#include "geodesy/wgs84.h"
#include "geodesy/utm.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"

#include "geofrenzy/gf_entitlement.h"
#include "swri_transform_util/local_xy_util.h"
#include "swri_transform_util/transform_util.h"

#define noop

/**
 * This node subscribes to a GPS signal on topic "fix".  It then uses that information to call the GeoFrenzy fencing agent to receive a series of
 * fences and entitlements based on the GPS location and a "zoom" level.  It then publishes the geofences as a occupancy grid map on topic "map",
 * the metadata as MapMetaData messages on topic "map_metadata", fence dwell information via topic "dwell" which is of locally defines type "geofrenzy/gf_entitlement"
 * It also exposes the service "static_map" which returns a occupancy grid map.
 * @todo Make the zoom level dynamic
 * @todo Allow for selection of specific entitlements for advertisement
 */

extern "C" char *ambient_fences_geojson_zoom(double lng, double lat, int lvl, int myclass);

class gf_fence_request
{
  public:
    double longitude; ///<longitude for GeoFrenzy Fence Request
    double latitude;  ///<latitude for GeoFrenzy Fence Request
    int zoom;         ///<Zoom level around longitude and latitude
};

class geojson_point
{
  public:
    double longitude; ///<longitude in goejson feature
    double latitude;  ///<latitude in geojson feature
};

class geojson_points
{
  public:
    std::vector<geojson_point> point; ///<point list in geosjson feature
};

class geojson_linear_rings
{
  public:
    std::vector<geojson_points> ring; ///<rings in geojson polygon
};

class geojson_geometry
{
  public:
    std::string type;                        ///< geojson geometry type
    std::vector<geojson_linear_rings> rings; ///<geojson array of rings comprising geojson polygon
};

class geojson_gfproperties
{
  public:
    std::string entitlement; ///<GeoFrenzy entitlement string for fence
    std::string inout;       ///<is the GPS locations submitted in the GoeFrenzy Fence request inside or outside this fence
                             ///< 'i' = within the fence; 'o' = outside the fence;
};

class geojson_feature
{
  public:
    std::string type;
    geojson_gfproperties properties; ///<geojson feature properties - should hold the GeoFrenzy metadata about the fence
    geojson_geometry geometry;       ///<geojson feature geometry
};

class geojson_feature_collection
{
  public:
    std::vector<geojson_feature> feature; ///<geojson feature in geojson feature collection
};

class geojson_root_fc
{
  public:
    std::string type;                    ///<geojson root type
    geojson_feature_collection features; ///<features list in geojson feature collection
};

class MapGrid
/**
 * This class holds the data used to create the occupancy map
 */

{
  public:
    int gridwidth;
    int gridlength;
    int gridsize;
    std::vector<signed char> grid;
    //int8_t  *grid;
    geojson_point origin;

    MapGrid(int width, int length);
};

MapGrid::MapGrid(int width, int length)
/**
 * This is the constructor for the MapGrid
 * \param width width in meters of the occupancy map
 * \param length length in meters of the occupancy map
 */
{
    gridwidth = (int)(width);
    gridlength = (int)(length);
    gridsize = gridwidth * gridlength;
    grid.assign(gridsize, 0);
}

class xy_coordinates
{
    /**
        * This class holds the Geojson point data once it's converted to a XY grid with an lat long anchor
        */

  public:
    double x;
    double y;
};

class xy_data
{
    /**
        * This class holds the geojson polygon data once it's converted to a XY grid with an lat long anchor
        */
  public:
    std::vector<xy_coordinates> coordinates;
    std::vector<geojson_point> original_ll;
};

class xy_feature
{
    /**
        * This class holds the Geojson feature data once it's converted to a XY grid with an lat long anchor
        */
  public:
    std::vector<xy_data> polygon;
    xy_coordinates xyanchor;
    geojson_point llanchor;
};

void geotosquare(double lat, double lon, double &x, double &y)
{

    /**
    * This function converts a lat long to a global XY frame
    * \param lat input global latitude
    * \param long input global longitude
    * \param x result global x location
    * \param y result global y location
    * @todo double check and look for better algorithms for this
    */
    double earth_cir = 40.057 * 1000000;

    double mapHeight = earth_cir;
    double mapWidth = earth_cir;

    x = (lon + 180) * (mapWidth / 360);
    double latRad = lat * M_PIl / 180;
    double mercN = log(tan((M_PIl / 4) + latRad / 2));
    y = (mapHeight / 2) - (lat * mapHeight) / 180;
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
        cout << x;
        if (steep)
        {
            grid.grid[(x * (int)grid.gridwidth) + (int)y] = (int8_t)100;
            grid.grid[(x * (int)grid.gridwidth) + (int)y + 1] = (int8_t)100;
        }
        else
        {
            grid.grid[(y * (int)grid.gridwidth) + (int)x] = (int8_t)100;
            grid.grid[(y * (int)grid.gridwidth) + (int)x + 1] = (int8_t)100;
        }

        error -= dy;
        if (error < 0)
        {
            y += ystep;
            error += dx;
        }
    }
}

// output contents of array to screen
void printArray(int8_t arr[], int size)
{
    for (int i = 0; i < size; i++)
    {
        cout << arr[i] << ' ';
    }
    cout << endl;
}

class MapServer

{
    /**
        * This class holds the map server that does most of the work to
        * convert the Geofrenzy FDN data to ROS messages and services
        * to be easily used by ROS nodes
        */

  public:
    void mapServerCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
    MapServer();
    bool mapCallback(nav_msgs::GetMap::Request &req,
                     nav_msgs::GetMap::Response &res);

  private:
    ros::Publisher pub;
    ros::NodeHandle n;
    ros::Publisher map_pub;
    ros::Publisher metadata_pub;
    ros::Publisher dwell_pub;
    bool deprecated;
    /** The map data is cached here, to be sent out to service callers*/
    nav_msgs::MapMetaData meta_data_message_;
    nav_msgs::GetMap::Response map_resp_;
    double_t previous_lat;
    double_t previous_long;
};

MapServer::MapServer(void)
{
    std::cout << "start MapServer Constuctor\n";
    std::string frame_id;
    ros::NodeHandle private_nh("~");
    private_nh.param("frame_id", frame_id, std::string("map"));
    map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    metadata_pub = n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    dwell_pub = n.advertise<geofrenzy::gf_entitlement>("dwell", 1, true);
    previous_lat = 0;
    previous_long = 0;
    std::cout << "end MapServer Construcotr\n";
}

void MapServer::mapServerCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    std::cout << "start MapServer Callback\n";
    xy_feature xy_features;
    Json::Value root; // will contains the root value after parsing.
    Json::Reader reader;

    std::cout << previous_lat;
    std::cout << previous_long;
    std::cout << "*******************************************************************latlong\n";
    std::cout << msg->latitude;
    std::cout << msg->longitude;

    if (msg->status.status == -1)
    {
        std::cout << "no gps acquired\n";
        // no gps acquired
        return;
    }

    double distance;

    distance = swri_transform_util::GreatCircleDistance(msg->latitude, msg->longitude, previous_lat, previous_long);
    std::cout << "distance=" << distance << "\n";

    //    if ((msg->longitude == previous_long) && (msg->latitude == previous_lat) ) {
    if (distance < 1)
    {
        return;
    }
    else
    {
        previous_lat = msg->latitude;
        previous_long = msg->longitude;
    }
    std::string filename;
    char *t;
    if (n.getParam("geojson_file", filename))
    {
        //char *buf;
        std::cout << "read file \n";
        std::ifstream in(filename.c_str());
        string message;
        while (in)
        {
            message.push_back(in.get());
        }
        char *bt = &message[0u];
        t = bt;
        //std::vector<char> buf(message.c_str(), message.c_str() + message.size() + 1);
        //t = buf;
        std::cout << "done read file \n";
        std::cout.flush();
    }
    else
    {
        char *td = ambient_fences_geojson_zoom(msg->longitude, msg->latitude, 4, 32);
        t = td;
    }
    std::cout << "begin *t\n";
    std::cout << t;
    std::cout << "end *t\n";
    std::cout.flush();

    bool parsingSuccessful = reader.parse(t, root);
    if (!parsingSuccessful)
    {
        // report to the user the failure and their locations in the document.
        std::cout << "Failed to parse configuration\n"
                  << reader.getFormattedErrorMessages();
        return;
    }

    geojson_root_fc gj_root;
    printf("gj_root\n");
    //Json::StyledWriter writer;
    std::cout << root;
    std::cout << "\n";
    gj_root.type = root["type"].asString();
    Json::Value features = root["features"];
    geojson_feature_collection gj_fc;
    printf("features\n");
    std::cout << "feature.len=";
    std::cout << features.size();
    std::cout << "\n";
    std::cout.flush();
    if (features.size() == 0)
    {
        std::cout << "no features!\n";
        return;
    }
    for (int findex = 0; findex < features.size(); findex++)
    {
        std::cout << "findex=";
        std::cout << findex;
        std::cout << "\n";
        std::cout.flush();
        geojson_feature gj_feature;
        gj_feature.type = features[findex]["type"].asString();
        std::cout << "gj_feature.type=";
        std::cout << gj_feature.type;
        std::cout << "\n";
        std::cout.flush();
        geojson_gfproperties gj_properties;
        try
        {
            gj_properties.entitlement = features[findex]["properties"]["entitlement"].asString();
            gj_properties.inout = features[findex]["properties"]["inout"].asString();
        }
        catch (int e)
        {
            cout << "no properties \n";
            std::cout.flush();
            return;
        }
        if (gj_properties.inout.compare("i") == 0)
        {
            geofrenzy::gf_entitlement entitlement_message;
            entitlement_message.entitlement = gj_properties.entitlement;
            entitlement_message.dwell = gj_properties.inout;
            entitlement_message.header.stamp = ros::Time::now();
            dwell_pub.publish(entitlement_message);
        }
        if (gj_properties.inout.compare("o") == 0)
        {
            geofrenzy::gf_entitlement entitlement_message;
            //entitlement_message.entitlement = gj_properties.entitlement;
            entitlement_message.dwell = gj_properties.inout;
            dwell_pub.publish(entitlement_message);
        }
        gj_feature.properties = gj_properties;

        cout << "entitlment=";
        cout << gj_feature.properties.entitlement;
        cout << "\ninout=";
        cout << gj_feature.properties.inout;
        cout << "\n";
        std::cout.flush();

        Json::Value geometry = features[findex]["geometry"];
        //   printf("geometry\n");
        geojson_geometry gj_geom;
        gj_geom.type = geometry["type"].asString();
        Json::Value coordinates = geometry["coordinates"];
        // printf("coordinates\n");
        for (int cindex = 0; cindex < coordinates.size(); ++cindex)
        {
            Json::Value rings = coordinates[cindex];
            printf("rings\n");
            geojson_linear_rings gj_ring_list;
            for (int rindex = 0; rindex < rings.size(); ++rindex)
            {
                Json::Value points = rings[rindex];
                // printf("points\n");
                //cout << points;
                geojson_points gj_point_list;
                geojson_point point;
                point.longitude = points[0].asDouble();
                point.latitude = points[1].asDouble();
                gj_point_list.point.push_back(point);

                gj_ring_list.ring.push_back(gj_point_list);
            }
            gj_geom.rings.push_back(gj_ring_list);
        }
        gj_feature.geometry = gj_geom;
        gj_fc.feature.push_back(gj_feature);
    }
    std::cout << "done walking features\n";
    std::cout.flush();
    gj_root.features = gj_fc;

    //   int j = 0;
    double xlow;
    double ylow;
    double xhigh;
    double yhigh;
    double zhigh;
    double zlow;
    double latlow;
    double longlow;
    double lathigh;
    double longhigh;
    std::vector<double> xpoints;
    std::vector<double> ypoints;
    std::vector<double> zpoints;

    //  std::vector<geojson_coordinates> coord_vector;

    std::vector<double> latvector;
    std::vector<double> longvector;

    // OK first iterator feature_collection
    for (std::vector<geojson_feature>::iterator it = gj_root.features.feature.begin();
         it != gj_root.features.feature.end();
         ++it)
    {
        xy_data xy_vector;
        geojson_feature feature = *it;
        geojson_geometry geometry = feature.geometry;
        // OK iterate rings

        for (std::vector<geojson_linear_rings>::iterator rit = geometry.rings.begin();
             rit != geometry.rings.end();
             ++rit)
        {
            geojson_linear_rings rings = *rit;
            cout << "rings";
            // cout << geometry;
            cout << "\n";
            cout.flush();

            std::vector<geojson_points> ring = rings.ring;
            // OK let's do two passes through
            // one to get the max/min lat long so we can define the frame anchor
            // two to adjust the data before sticking it in the grid
            // here we go with finding the frame anchor
            for (std::vector<geojson_points>::iterator gpit = ring.begin(); gpit != ring.end(); ++gpit)
            {
                //    geojson_points anchor = *gpit;
                geojson_points points = *gpit;
                for (std::vector<geojson_point>::iterator pit = points.point.begin();
                     pit != points.point.end(); ++pit)
                {
                    //  double latanchor = anchor.latitude;
                    //  double longanchor = anchor.longitude;
                    double x;
                    double y;
                    geojson_point location = *pit;
                    latvector.push_back(location.latitude);
                    longvector.push_back(location.longitude);
                    xy_coordinates xylocation;
                    geotosquare(location.latitude, location.longitude, xylocation.x, xylocation.y);
                    xy_vector.coordinates.push_back(xylocation);
                    xy_vector.original_ll.push_back(location);
                }
            }
        }
        xy_features.polygon.push_back(xy_vector);
    }
    cout << "end xy feature conversion";
    // cout << geometry;
    cout << "\n";
    cout.flush();
    latlow = *min_element(latvector.begin(), latvector.end());
    longlow = *min_element(longvector.begin(), longvector.end());
    //    latlow = msg->latitude;
    //    longlow = msg->longitude;
    lathigh = *max_element(latvector.begin(), latvector.end());
    longhigh = *max_element(longvector.begin(), longvector.end());
    double anchorx;
    double anchory;
    xy_features.llanchor.latitude = lathigh;
    xy_features.llanchor.longitude = longlow;
    geotosquare(lathigh, longlow, xy_features.xyanchor.x, xy_features.xyanchor.y);
    printf("\nanchorx=%lf\n", xy_features.xyanchor.x);
    printf("\nanchory=%lf\n", xy_features.xyanchor.y);

    // OK now we have the upper left corner of the frame

    cout << "start data adjustment";
    // cout << geometry;
    cout << "\n";
    cout.flush();

    // now to adjust the data
    for (std::vector<xy_data>::iterator fit = xy_features.polygon.begin();
         fit != xy_features.polygon.end(); ++fit)
    {
        xy_data points = *fit;
        for (std::vector<xy_coordinates>::iterator pit = points.coordinates.begin();
             pit != points.coordinates.end(); ++pit)
        {
            xy_coordinates location = *pit;
            xy_coordinates newlocation;

            // ok let's shift everything from a global frame to a local frame
            // TODO: double check the shift by ten
            newlocation.x = location.x - xy_features.xyanchor.x;
            +10;
            newlocation.y = location.y - xy_features.xyanchor.y;
            +10;

            xpoints.push_back(location.x);
            ypoints.push_back(location.y);
            printf("x=%lf,newx=%lf\n", location.x, newlocation.x);
            printf("y=%lf,newy=%lf\n", location.y, newlocation.y);
            *pit = newlocation;
        }
        *fit = points;
    }

    cout << "end data adjustment";
    // cout << geometry;
    cout << "\n";
    cout.flush();

    // OK we are done with the overall calculations now we need to  use them to build the map

    xlow = *min_element(xpoints.begin(), xpoints.end());
    ylow = *min_element(ypoints.begin(), ypoints.end());
    xhigh = *max_element(xpoints.begin(), xpoints.end());
    yhigh = *max_element(ypoints.begin(), ypoints.end());
    // try to expand map so that it edges and vertex done touch the edge of the bounding box.
    double width = xhigh - xlow + 20;
    double length = yhigh - ylow;
    +20;

    printf("\nxlow=%lf\n", xlow);
    printf("\nylow=%lf\n", ylow);
    printf("\nxhigh=%lf\n", xhigh);
    printf("\nyhigh=%lf\n", yhigh);
    printf("\nwidth=%lf\n", width);
    printf("\nlength=%lf\n", length);
    printf("\nlatlow=%lf\n", latlow);
    printf("\nlonglow=%lf\n", longlow);
    printf("\nlathigh=%lf\n", lathigh);
    printf("\nlonghigh=%lf\n", longhigh);

    // OK now that all of the data is together need to put it all in one place
    // latlow and longlow = 0,0
    // lat ~ Y
    // long ~ X
    // lat++ => north
    // long++ => east
    // lat = 90 ; long = 180  ===  x = 0 ; y = 0

    cout << "start building grid";
    // cout << geometry;
    cout << "\n";
    cout.flush();

    /////
    //swri_transform_util::LocalXyWgs84Util localtest = swri_transform_util::LocalXyWgs84Util(lathigh,longlow,0,0);

    ////

    MapGrid grid((int)width, (int)length);
    grid.origin.latitude = lathigh;
    grid.origin.longitude = longlow;

    for (std::vector<xy_data>::iterator oit = xy_features.polygon.begin();
         oit != xy_features.polygon.end(); ++oit)
    {
        xy_data xy_line_vector = *oit;
        printf("xy_line_vector length = %ld\n", xy_line_vector.coordinates.size());
        /////////////////////////////////////

        for (std::vector<geojson_point>::iterator lt = xy_line_vector.original_ll.begin();
             lt != xy_line_vector.original_ll.end(); ++lt)
        {
            geojson_point ll = *lt;
            std::vector<geojson_point>::iterator dupe = lt;
            ++dupe;
            geojson_point b = *dupe;
            if ((ll.latitude == b.latitude) and (ll.longitude == b.longitude))
            {
                printf("zero\n");
                // 0 length segment no point in drawing it
                noop;
            }
            else if (dupe == xy_line_vector.original_ll.end())
            {
                // at end of vector, b has no meaning
                printf("end\n");
                noop;
            }
            else
            {
                std::cout << "ll.latitude=" << ll.latitude << "\n";
                std::cout << "ll.longitude=" << ll.longitude << "\n";
                double tempx;
                double tempy;
                double btempx;
                double btempy;
                swri_transform_util::LocalXyFromWgs84(ll.latitude, ll.longitude, latlow, longlow, tempx, tempy);
                swri_transform_util::LocalXyFromWgs84(b.latitude, b.longitude, latlow, longlow, btempx, btempy);
                std::cout << "tempx=" << tempx << " tempy=" << tempy << "\n";
                std::cout << "btempx=" << btempx << " btempy=" << btempy << "\n";
                std::cout.flush();
                // cout << grid.grid;
                Line(tempx, tempy, btempx, btempy, grid);
            }
        }
        ///////////////////////////////////
        //   for (std::vector<xy_coordinates>::iterator it = xy_line_vector.coordinates.begin();
        //         it != xy_line_vector.coordinates.end(); ++it) {
        //   xy_coordinates a = *it;
        //    std::vector<xy_coordinates>::iterator dupe = it;
        //   ++dupe;
        //  std::cout << "The distance is: " << std::distance(it,dupe) << '\n';
        //  xy_coordinates b = *dupe;
        //  if ((a.x == b.x) and (a.y == b.y))
        //  {
        //      printf("zero\n");
        //      // 0 length segment no point in drawing it
        //      noop;
        //  }
        //   else if (dupe == xy_line_vector.coordinates.end())
        //   {
        //       // at end of vector, b has no meaning
        //       printf("end\n");
        //       noop;
        //   }
        //~ else {
        //~ // cout << grid.grid;
        //~ Line(a.x, a.y, b.x, b.y, grid);
        //~ printf("a.x=%lf,", a.x);
        //~ printf("a.y=%lf\n", a.y);
        //~ printf("b.x=%lf,", b.x);
        //~ printf("b.y=%lf\n\n", b.y);
        //~ }
        //  }

        printf("polygon end\n");
    }
    std::cout << "start bulding ros messages";
    // cout << geometry;
    std::cout << "\n";
    std::cout.flush();
    // let's define the occ
    ros::Time current_time;
    nav_msgs::OccupancyGrid ogrid;
    map_resp_.map.info.map_load_time = ros::Time::now();
    map_resp_.map.header.frame_id = "map";
    map_resp_.map.header.stamp = ros::Time::now();
    cout << "header done";
    // cout << geome
    cout << "\n";
    cout.flush();
    map_resp_.map.info.resolution = 1.0;
    map_resp_.map.info.width = grid.gridwidth;
    map_resp_.map.info.height = grid.gridlength;
    // map_resp_.map.info.origin.position.x = grid.gridwidth/2;
    // map_resp_.map.info.origin.position.y = grid.gridlength/2;
    //  map_resp_.map.info.origin.orientation.z = 0;
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    map_resp_.map.info.origin.orientation.x = q.x();
    map_resp_.map.info.origin.orientation.y = q.y();
    map_resp_.map.info.origin.orientation.z = q.z();
    map_resp_.map.info.origin.orientation.w = q.w();
    cout << "pose done";
    // cout << geometry;
    cout << "\n";

    cout << grid.grid.size();
    cout.flush();
    meta_data_message_ = map_resp_.map.info;
    cout << "meta_data_messages";
    cout << "\n";
    cout.flush();
    //std::vector<signed char> g(grid.grid, grid.grid + (grid.gridsize));
    //std::vector<signed char> g(std::begin(grid.grid), std::end(grid.grid));
    //int8_t* tempgrid = &grid.grid[0];

    //signed char j , tempgrid[grid.grid.size()];
    //j=0;
    std::cout << "max_size:" << map_resp_.map.data.max_size() << "\n";
    std::cout << "max_size:" << map_resp_.map.data.max_size() << "\n";
    std::cout.flush();
    //map_resp_.map.data.resize(grid.grid.size());
    //for  (std::vector<signed char>::const_iterator i = grid.grid.begin(); i != grid.grid.end(); ++i)
    //{

    //std::cout << "capacity:" << map_resp_.map.data.capacity() << "\n";
    //std::cout << "size:" << map_resp_.map.data.size() << "\n";
    //std::cout.flush();
    //map_resp_.map.data.push_back(*i);
    //}
    //map_resp_.map.data = tempgrid;
    //std::copy(tempgrid.begin(), tempgrid.end(), map_resp_.map.data);
    //std::memcpy(tempgrid,grid.grid,sizeof tempgrid);
    //map_resp_.map.data = &tempgrid[0];
    //map_resp_.map.data = &tempgrid;
    //std::memcpy(map_resp_.map.data,grid.grid,sizeof map_resp_.map.data);
    map_resp_.map.data = grid.grid;
    // map_resp_.map.data.swap(grid.grid);
    cout << "grid.grid";
    cout << "\n";
    cout.flush();
    metadata_pub.publish(meta_data_message_);
    cout << "metadata published";
    // cout << geometry;
    cout << "\n";
    cout.flush();
    map_pub.publish(map_resp_.map);
    cout << "done";
    // cout << geometry;
    cout << "\n";
    cout.flush();
};

/** Callback invoked when someone requests our service */
bool MapServer::mapCallback(nav_msgs::GetMap::Request &req,
                            nav_msgs::GetMap::Response &res)
{
    ROS_INFO("Requestin Map");
    // request is empty; we ignore it

    // = operator is overloaded to make deep copy (tricky!)
    res = map_resp_;
    ROS_INFO("Sending map");

    return true;
};

/*
void metadataSubscriptionCallback(const ros::SingleSubscriberPublisher& pub)
{
  pub.publish( meta_data_message_ );
}
*/

void geotoxyz(double lat, double lon, double &x, double &y, double &z)
{
    double rad = 6378137.0;
    ;
    double f = 1.0 / 298.257224;
    double h = 0.0;
    double cosLat = cos(lat * M_PI / 180.0);
    double sinLat = sin(lat * M_PI / 180.0);
    double cosLon = cos(lon * M_PI / 180.0);
    double sinLon = sin(lon * M_PI / 180.0);
    double C = 1.0 / sqrt(cosLat * cosLat + (1 - f) * (1 - f) * sinLat * sinLat);
    double S = (1.0 - f) * (1.0 - f) * C;
    x = (rad * C + h) * cosLat * cosLon;
    y = (rad * C + h) * cosLat * sinLon;
    z = (rad * S + h) * sinLat;
}

void xyztogeo(double x, double y, double z, double &lat, double &lon)
{
    double a = 6378137.0;
    double e = 8.1819190842622e-2;
    double asq = pow(a, 2);
    double esq = pow(e, 2);

    double b = sqrt(asq * (1 - esq));
    double bsq = pow(b, 2);
    double ep = sqrt((asq - bsq) / bsq);
    double p = sqrt(pow(x, 2) + pow(y, 2));
    double th = atan2(a * z, b * p);

    lon = atan2(y, x);
    lat = atan2((z + pow(ep, 2) * b * pow(sin(th), 3)),
                (p - esq * a * pow(cos(th), 3)));
    double N = a / (sqrt(1 - esq * pow(sin(lat), 2)));
    double alt = p / cos(lat) - N;
    // mod lat to 0-2pi
    lon = fmod(lon, (2 * M_PI));
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gf_map_server", ros::init_options::AnonymousName);

    //gf_fence_request gpsfix;
    ros::NodeHandle n;
    ros::Subscriber gps;


    try
    {

        MapServer ms;
        ros::ServiceServer service;
        gps = n.subscribe("fix", 1, &MapServer::mapServerCallback, &ms);
        printf("gps subscribed\n");
        service = n.advertiseService("static_map", &MapServer::mapCallback, &ms);
        printf("static_map service advertised\n");
        ros::spin();
    }
    catch (std::exception &e)
    {
        std::cout << "Error: " << e.what() << "\n";
    }
    return 0;
}
