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

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"
#include <tf/tf.h>

#include <cmath>
#include <iostream>
#include <string>
using namespace std;
#include <ctime>
#include <cstdlib>
#include <algorithm>

#include <vector>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <set>
#include <exception>
#include <iostream>
#include <limits>
#include <iterator>

#include <string>
#include <fstream>
#include <streambuf>
#include "jsoncpp/json/json.h"
#include <iostream>


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geodesy/wgs84.h"
#include "geodesy/utm.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"

#define noop

class MapGrid
{

    public:
        int gridwidth ;
        int gridlength ;
        int gridsize ;
        int8_t  * grid;

        MapGrid(int width, int length);
};

MapGrid::MapGrid( int width,  int length )
{
    gridwidth = (int)width + 100;
    gridlength = (int)length + 100;
    gridsize = gridwidth * gridlength;
    grid = new int8_t[gridsize];
    std::fill_n(grid, gridsize, 0);
}


class MapServer

{
    public:
        MapServer(const std::string& filename) {
            std::string frame_id;
            ros::NodeHandle private_nh("~");
            private_nh.param("frame_id", frame_id, std::string("map"));


            Json::Value root;   // will contains the root value after parsing.
            Json::Reader reader;
            std::ifstream t(filename.c_str());
            bool parsingSuccessful = reader.parse( t, root );
            if ( !parsingSuccessful )
            {
                // report to the user the failure and their locations in the document.
                std::cout  << "Failed to parse configuration\n"
                           << reader.getFormattedErrorMessages();
                return ;
            }

            geojson_root_fc gj_root;
            printf("gj_root\n");

            gj_root.type = root["type"].asString();
            Json::Value features = root["features"];
            geojson_feature_collection gj_fc;
            printf("features\n");
            for (int findex = 0; findex < features.size(); ++findex)
            {
                geojson_feature gj_feature;
                gj_feature.type = features[findex]["type"].asString();
                gj_feature.properties = "PLACEHOLDER";
                Json::Value geometry = features[findex]["geometry"];
                printf("geometry\n");
                geojson_geometry gj_geom;
                gj_geom.type=geometry["type"].asString();
                Json::Value coordinates = geometry["coordinates"];
                printf("coordinates\n");
                for (int cindex = 0; cindex < coordinates.size(); ++cindex)
                {
                    Json::Value rings = coordinates[cindex];
                    printf("rings\n");
                    geojson_poly_rings gj_ring_list;
                    for (int rindex = 0; rindex < rings.size(); ++rindex) {
                        Json::Value points = rings[rindex];
                        printf("points\n");
                        cout << points;
                        geojson_points gj_point_list;
                        geojson_point point;
                        point.longitude = points[0].asDouble();
                        point.latitude = points[1].asDouble();
                        gj_point_list.point.push_back(point);

                        gj_ring_list.ring.push_back(gj_point_list);
                    }
                    gj_geom.coordinates.rings.push_back(gj_ring_list);
                }
                gj_feature.geometry = gj_geom;
                gj_fc.feature.push_back(gj_feature);
            }
            gj_root.features = gj_fc;




            int j = 0;
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

            std::vector<geojson_coordinates> coord_vector;

            xy_feature xy_features;


            // OK first iterator feature_collection
            for (std::vector<geojson_feature>::iterator it = gj_root.features.feature.begin();
                    it != gj_root.features.feature.end();
                    ++it)
            {
                xy_data xy_vector;
                geojson_feature  feature = *it;
                printf("type=%s\n",feature.type.c_str());
                geojson_geometry geometry = feature.geometry;
                // OK iterate rings
                for (std::vector<geojson_poly_rings>::iterator rit = geometry.coordinates.rings.begin();
                        rit != geometry.coordinates.rings.end();
                        ++rit) {
                    geojson_poly_rings rings = *rit;
                    std::vector<double> latvector;
                    std::vector<double> longvector;
                    std::vector<geojson_points> ring = rings.ring;
                    for (std::vector<geojson_points>::iterator gpit = ring.begin(); gpit != ring.end(); ++ gpit)
                    {
                        //    geojson_points anchor = *gpit;
                        geojson_points points = *gpit;

                        // OK let's do two passes through
                        // one to get the max/min lat long so we can define the frame anchor
                        // two to adjust the data before sticking it in the grid
                        // here we go with finding the frame anchor
                        for (std::vector<geojson_point>::iterator pit = points.point.begin();
                                pit != points.point.end(); ++pit) {
                            //  double latanchor = anchor.latitude;
                            //  double longanchor = anchor.longitude;
                            geojson_point location = *pit;
                            latvector.push_back(location.latitude);
                            longvector.push_back(location.longitude);
                        }
                        latlow = *min_element(latvector.begin(), latvector.end());
                        longlow = *min_element(longvector.begin(), longvector.end());
                        lathigh = *max_element(latvector.begin(), latvector.end());
                        longhigh = *max_element(longvector.begin(), longvector.end());
                        double anchorx;
                        double anchory;
                        double anchorlat = lathigh;
                        double anchorlong = longlow;

                        geotosquare(lathigh, longlow, anchorx, anchory);
                        printf("\nanchorx=%lf\n", anchorx);
                        printf("\nanchory=%lf\n", anchory);
                        // OK now we have the upper left corner of the frame




                        for (std::vector<geojson_point>::iterator pit = points.point.begin();
                                pit != points.point.end(); ++pit) {
                            double x;
                            double y;

                            geojson_point location = *pit;
                            xy_coordinates xylocation;
                            geotosquare(location.latitude, location.longitude, x, y);

                            // ok let's shift everything from a global frame to a local frame
                            x = x - anchorx;
                            y = y - anchory;

                            xpoints.push_back(x);
                            ypoints.push_back(y);
                            printf("x=%lf\n", x);
                            printf("y=%lf\n", y);
                            xylocation.x = x;
                            xylocation.y = y;
                            xy_vector.coordinates.push_back(xylocation);
                        }
                    }
                }
                xy_features.polygon.push_back(xy_vector);
            }

            // OK we are done with the overall calculations now we need to  use them to build the map

            xlow = *min_element(xpoints.begin(), xpoints.end());
            ylow = *min_element(ypoints.begin(), ypoints.end());
            xhigh = *max_element(xpoints.begin(), xpoints.end());
            yhigh = *max_element(ypoints.begin(), ypoints.end());
            double width = xhigh - xlow;
            double length = yhigh - ylow;

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


            MapGrid grid((int)width,(int)length);

            for (std::vector<xy_data>::iterator oit = xy_features.polygon.begin();
                    oit != xy_features.polygon.end(); ++oit) {
                xy_data xy_vector = *oit;
                for (std::vector<xy_coordinates>::iterator it = xy_vector.coordinates.begin();
                        it != xy_vector.coordinates.end(); ++it) {
                    xy_coordinates a = *it;
                    std::vector<xy_coordinates>::iterator dupe = it;
                    ++dupe;
                    xy_coordinates b = *dupe;
                    if ((a.x == b.x) and (a.y == b.y))
                        // 0 length segment no point in drawing it
                        noop;
                    else if (dupe == xy_vector.coordinates.end())
                        // at end of vector, b has no meaning
                        noop;
                    else {
                        Line(a.x, a.y, b.x, b.y, grid);
                        printf("a.x=%lf,", a.x);
                        printf("a.y=%lf\n", a.y);
                        printf("b.x=%lf,", b.x);
                        printf("b.y=%lf\n\n", b.y);
                    }
                }

                printf("polygon end\n");
            }
            // let's define the occ
            ros::Time current_time;
            //   current_time = ros::Time::now();
            nav_msgs::OccupancyGrid ogrid;
            // nav_msgs::MapMetaData info;
            // geometry_msgs::Pose origin;
            map_resp_.map.info.map_load_time = ros::Time::now();
            map_resp_.map.header.frame_id = "map";
            map_resp_.map.header.stamp = ros::Time::now();
            //  map_resp_.map.header.frame_id = "map";
            map_resp_.map.info.resolution = 1.0;
            map_resp_.map.info.width = grid.gridwidth;
            map_resp_.map.info.height = grid.gridlength;
            map_resp_.map.info.origin.position.x = -1;
            map_resp_.map.info.origin.position.y = -1;
            map_resp_.map.info.origin.orientation.z = 0;
            tf::Quaternion q;
            q.setRPY(0, 0, 0);
            map_resp_.map.info.origin.orientation.x = q.x();
            map_resp_.map.info.origin.orientation.x = q.y();
            map_resp_.map.info.origin.orientation.x = q.z();
            map_resp_.map.info.origin.orientation.x = q.w();

            meta_data_message_ = map_resp_.map.info;

            std::vector<signed char> g(grid.grid, grid.grid + (grid.gridsize));
            map_resp_.map.data = g;

            service = n.advertiseService("static_map", &MapServer::mapCallback, this);

            metadata_pub = n.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
            metadata_pub.publish(meta_data_message_);

            map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
            map_pub.publish( map_resp_.map );

        };




    private:
        ros::Publisher pub;

        struct geojson_point {
            double longitude;
            double latitude;
        };

        struct geojson_points {
            std::vector<geojson_point> point;
        };

        struct geojson_poly_rings {
            std::vector<geojson_points> ring;
        };

        struct geojson_coordinates {
            std::vector<geojson_poly_rings> rings;
        };

        struct geojson_geometry {
            std::string type;
            geojson_coordinates coordinates;
        };

        struct geojson_feature {
            std::string type;
            std::string properties;  ////// PLACEHOLDER UNTIL WE START PASSING MORE INFO
            geojson_geometry geometry;
        };

        struct geojson_feature_collection {
            std::vector<geojson_feature> feature;
        };

        struct geojson_root_fc {
            std::string type;
            geojson_feature_collection features;
        };

        struct xy_coordinates {
            double x;
            double y;
        };

        struct xy_data {
            std::vector<xy_coordinates> coordinates;
        };

        struct xy_feature {
            std::vector<xy_data> polygon;
        };

        ros::NodeHandle n;
        ros::Publisher map_pub;
        ros::Publisher metadata_pub;
        ros::ServiceServer service;
        bool deprecated;

        /** Callback invoked when someone requests our service */
        bool mapCallback(nav_msgs::GetMap::Request & req,
                         nav_msgs::GetMap::Response & res) {
            // request is empty; we ignore it

            // = operator is overloaded to make deep copy (tricky!)
            res = map_resp_;
            ROS_INFO("Sending map");

            return true;
        };

        /** The map data is cached here, to be sent out to service callers*/
        nav_msgs::MapMetaData meta_data_message_;
        nav_msgs::GetMap::Response map_resp_;

        /*
        void metadataSubscriptionCallback(const ros::SingleSubscriberPublisher& pub)
        {
          pub.publish( meta_data_message_ );
        }
        */

        //using namespace std;
        //using boost::property_tree::ptree;

        void geotosquare(double lat, double lon, double& x, double& y) {
            ;
            double earth_cir = 40.057 * 1000000;

            double mapHeight = earth_cir;
            double mapWidth = earth_cir;

            x = (lon + 180) * (mapWidth / 360);
            double latRad = lat * M_PIl / 180;
            double mercN = log(tan((M_PIl / 4) + latRad / 2));
            // y = (mapHeight/2) -(mapHeight*mercN/(2*M_PIl));
            y = (mapHeight / 2) - (lat * mapHeight) / 180;

            // y = (mapHeight/2)-(mapHeight*mercN/(2*PI))

            //  double adjustedlat = lat - latmin
            //  double scaledlat = lat * 1000000;
            //  double scaledlon = lon * 1000000;
        }

        void geotoxyz(double lat, double lon, double& x, double& y, double& z) {
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

        void xyztogeo(double x, double y, double z, double& lat, double& lon) {
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
        }

        void Line(double x1, double y1, double x2, double y2, MapGrid grid)
        {
            // Bresenham's line algorithm
            const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
            if (steep) {
                std::swap(x1, y1);
                std::swap(x2, y2);
            }

            if (x1 > x2) {
                std::swap(x1, x2);
                std::swap(y1, y2);
            }

            const double dx = x2 - x1;
            const double dy = fabs(y2 - y1);

            float error = dx / 2.0f;
            const int ystep = (y1 < y2) ? 1 : -1;
            int y = (int)y1;

            const int maxX = (int)x2;

            for (int x = (int)x1; x < maxX; x++) {
                if (steep) {
                    //  SetPixel(y,x);
                    grid.grid[(x * (int)grid.gridwidth) + (int)y] = 100;
                } else {
                    grid.grid[(y * (int)grid.gridwidth) + (int)x] = 100;
                }

                error -= dy;
                if (error < 0) {
                    y += ystep;
                    error += dx;
                }
            }
        }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gf_map_server", ros::init_options::AnonymousName);
    try {
        MapServer ms("two.json");
        ros::spin();
    } catch (std::exception& e) {
        std::cout << "Error: " << e.what() << "\n";
    }
    return 0;
}













