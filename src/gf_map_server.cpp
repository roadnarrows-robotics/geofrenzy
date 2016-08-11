// ----------------------------------------------------------------------------
// Copyright (C) 2002-2006 Marcin Kalicinski
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
// For more information, see www.boost.org
// ----------------------------------------------------------------------------
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
#include  <limits>
#include <iterator>


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geodesy/wgs84.h"
#include "geodesy/utm.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/OccupancyGrid.h"

#define noop

ros::Publisher pub;




struct geojson_coordinates
{
    double longitude;
    double latitude;
};

struct geojson_data
{
    std::vector<geojson_coordinates> coordinates;
};

struct xy_coordinates
{
    double x;
    double y;
};

struct xy_data
{
    std::vector<xy_coordinates> coordinates;
};


using namespace std;
using boost::property_tree::ptree;

void geotosquare( double lat, double lon,
                  double& x, double& y)
{   ;
    double earth_cir = 40.057 * 1000000;

    double mapHeight = earth_cir;
    double mapWidth = earth_cir;

    x = (lon+180)*(mapWidth/360);
    double latRad = lat*M_PIl/180;
    double mercN = log(tan((M_PIl/4)+latRad/2));
    // y = (mapHeight/2) -(mapHeight*mercN/(2*M_PIl));
    y = (mapHeight/2) - (lat * mapHeight)/180;

    //y = (mapHeight/2)-(mapHeight*mercN/(2*PI))

    //  double adjustedlat = lat - latmin
    //  double scaledlat = lat * 1000000;
    //  double scaledlon = lon * 1000000;

}

void geotoxyz ( double lat, double lon,
                double& x, double& y, double& z)
{
    double rad = 6378137.0;;
    double f = 1.0 / 298.257224;
    double h = 0.0;
    double cosLat = cos(lat * M_PI / 180.0);
    double sinLat = sin(lat * M_PI /180.0);
    double cosLon = cos(lon * M_PI / 180.0);
    double sinLon = sin(lon * M_PI /180.0);
    double C = 1.0 / sqrt(cosLat * cosLat + (1 - f) * (1 - f) * sinLat * sinLat);
    double S = (1.0 - f) * (1.0 - f) * C;
    x = (rad * C + h) * cosLat * cosLon;
    y = (rad * C + h) * cosLat * sinLon;
    z =  (rad * S + h) * sinLat;
}


void xyztogeo (double x, double y, double z,
               double& lat, double& lon)
{
    double a = 6378137.0;
    double e = 8.1819190842622e-2;
    double asq = pow(a,2);
    double esq = pow(e,2);

    double b = sqrt( asq * (1-esq) );
    double bsq = pow(b,2);
    double ep = sqrt( (asq - bsq)/bsq);
    double p = sqrt( pow(x,2) + pow(y,2) );
    double th = atan2(a*z, b*p);

    lon = atan2(y,x);
    lat = atan2( (z + pow(ep,2)*b*pow(sin(th),3) ), (p - esq*a*pow(cos(th),3)) );
    double N = a/( sqrt(1-esq*pow(sin(lat),2)) );
    double alt = p / cos(lat) - N;
    // mod lat to 0-2pi
    lon = fmod(lon , (2*M_PI));

}

void Line( double x1, double y1, double x2, double y2 , int width, int height, int8_t grid[])
{
    // Bresenham's line algorithm
    const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
    if(steep)
    {
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    if(x1 > x2)
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

    for(int x=(int)x1; x<maxX; x++)
    {
        if(steep)
        {
            //  SetPixel(y,x);
            grid[(x*(int)width)+(int)y]=100;
        }
        else
        {
            grid[(y*(int)width)+(int)x]=100;
        }

        error -= dy;
        if(error < 0)
        {
            y += ystep;
            error += dx;
        }
    }
}




string indent(int level) {
    string s;
    for (int i=0; i<level; i++) s += "  ";
    return s;
}

void printTree (ptree &pt, int level) {
    if (pt.empty()) {
        cerr << "\""<< pt.data()<< "\"";
    } else {
        if (level) cerr << endl;
        cerr << indent(level) << "{" << endl;
        for (ptree::iterator pos = pt.begin(); pos != pt.end();) {
            cerr << indent(level+1) << "\"" << pos->first << "\": ";
            printTree(pos->second, level + 1);
            ++pos;
            if (pos != pt.end()) {
                cerr << ",";
            }
            cerr << endl;
        }
        cerr << indent(level) << " }";
    }
    return;
}

void load(const std::string &filename)
{
    ros::Rate loop_rate(10);

    // Create empty property tree object
    using boost::property_tree::ptree;
    ptree pt;

    // Load json file and put its contents in property tree.
    // No namespace qualification is needed, because of Koenig
    // lookup on the second argument. If reading fails, exception
    // is thrown.
    read_json(filename, pt);


    int j=0;
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
    std::vector <double> xpoints;
    std::vector <double> ypoints;
    std::vector <double> zpoints;

    std::vector <geojson_coordinates> coord_vector;
    std::vector <xy_coordinates> xy_vector;
    
        ptree ww = pt.get_child("features");
    // printTree(ww, 4);
    for (ptree::iterator pos = ww.begin(); pos != ww.end();++pos) {
 
    printTree(pos->second,4);
    printf("break\n");
   }

    BOOST_FOREACH(ptree::value_type &v, ww.get_child(".geometry.coordinates.."))
    {
        double coordinates[2];
        int i=0;
        BOOST_FOREACH(ptree::value_type &vv, v.second.get_child(""))
        {
            string test;
            test=vv.second.get<string>("");
            printf("%s",test.c_str());
            coordinates[i] = strtod(test.c_str(),NULL);
            printf("|%lf|\n",coordinates[i]);
            i++;

        };

        geojson_coordinates coords;
        geojson_data polygon;
        coords.longitude = coordinates[0];
        coords.latitude = coordinates[1];
        coord_vector.push_back(coords);

        geographic_msgs::GeoPoint ll;
        ll.latitude = coordinates[1];
        ll.longitude = coordinates[0];
        ll.altitude = std::numeric_limits<double>::quiet_NaN();
        //     geodesy::UTMPoint utmpoints(ll);
        //     printf("%d",utmpoints.zone);
        //     geometry_msgs::Point mypoints = geodesy::toGeometry(utmpoints);

        ros::spinOnce();
        loop_rate.sleep();

    };

    geojson_coordinates anchor = *coord_vector.begin();
    std::vector<double> latvector;
    std::vector<double> longvector;
    // OK let's do two passes through
    // one to get the max/min lat long so we can define the frame anchor
    // two to adjust the data before sticking it in the grid
    // here we go with finding the frame anchor
    for(std::vector<geojson_coordinates>::iterator it = coord_vector.begin(); it != coord_vector.end(); ++it)
    {
        double latanchor = anchor.latitude;
        double longanchor = anchor.longitude;
        geojson_coordinates location = *it;
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

    geotosquare(lathigh,longlow,anchorx,anchory);
    printf("\nanchorx=%lf\n",anchorx);
    printf("\nanchory=%lf\n",anchory);
    // OK now we have the upper left corner of the frame


    for(std::vector<geojson_coordinates>::iterator it = coord_vector.begin(); it != coord_vector.end(); ++it)
    {

        double x;
        double y;

        geojson_coordinates location = *it;
        xy_coordinates xylocation;
        geotosquare(location.latitude,location.longitude,x,y);

        // ok let's shift everything from a global frame to a local frame
        x = x - anchorx;
        y = y - anchory;
    

        xpoints.push_back(x);
        ypoints.push_back(y);
        printf("x=%lf\n",x);
        printf("y=%lf\n",y);
        xylocation.x=x;
        xylocation.y=y;
        xy_vector.push_back(xylocation);
    }

    xlow = *min_element(xpoints.begin(), xpoints.end());
    ylow = *min_element(ypoints.begin(), ypoints.end());
    xhigh = *max_element(xpoints.begin(), xpoints.end());
    yhigh = *max_element(ypoints.begin(), ypoints.end());
    double width = xhigh - xlow;
    double length = yhigh - ylow;

    printf("\nxlow=%lf\n",xlow);
    printf("\nylow=%lf\n",ylow);
    printf("\nxhigh=%lf\n",xhigh);
    printf("\nyhigh=%lf\n",yhigh);
    printf("\nwidth=%lf\n",width);
    printf("\nlength=%lf\n",length);
    printf("\nlatlow=%lf\n",latlow);
    printf("\nlonglow=%lf\n",longlow);
    printf("\nlathigh=%lf\n",lathigh);
    printf("\nlonghigh=%lf\n",longhigh);

    // OK now that all of the data is together need to put it all in one place
    // latlow and longlow = 0,0
    // lat ~ Y
    // long ~ X
    // lat++ => north
    //long++ => east
    // lat = 90 ; long = 180  ===  x = 0 ; y = 0

    int gridwidth = (int)width + 1;
    int gridlength = (int)length + 1;
    int gridsize= gridwidth*gridlength;
    printf("gridwidth=%d\n",gridwidth);
    printf("gridlength=%d\n",gridlength);
    printf("gridsize=%d\n",gridsize);
    int8_t grid[gridsize];
    std::fill_n(grid,gridsize,0);


    for(std::vector<xy_coordinates>::iterator it = xy_vector.begin(); it != xy_vector.end(); ++it)
    {
        xy_coordinates a = *it;
        std::vector<xy_coordinates>::iterator dupe=it;
        ++dupe;
        xy_coordinates b = *dupe;
        if ((a.x == b.x) and (a.y == b.y)) 
            //0 length segment no point in drawing it
            noop;
        else if (dupe == xy_vector.end())
            // at end of vector, b has no meaning
            noop;
        else
        {
            Line(a.x,a.y,b.x,b.y,gridwidth,gridlength,grid);
            printf("a.x=%lf,",a.x);
            printf("a.y=%lf\n",a.y);
            printf("b.x=%lf,",b.x);
            printf("b.y=%lf\n\n",b.y);
        }    
    }
    // let's define the occ
    ros::Time current_time;
    current_time = ros::Time::now();
    nav_msgs::OccupancyGrid ogrid;
    //nav_msgs::MapMetaData info;
    //geometry_msgs::Pose origin;
    ogrid.header.stamp = current_time;
    ogrid.header.frame_id = "gfframe";
    ogrid.info.map_load_time = current_time;
    ogrid.info.resolution = 1.0;
    ogrid.info.width = gridwidth;
    ogrid.info.height = gridlength;
    ogrid.info.origin.position.x = 0;
    ogrid.info.origin.position.y = length;
    std::vector<signed char> g(grid,grid + (gridsize));
    ogrid.data=g;
    pub.publish(ogrid);
    //ogrid.info.origin.position.z = 0;
    //ogrid.info.origin.orientation.
    loop_rate.sleep();
    
    
    
    
}


int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "talker");
        ros::NodeHandle n;
        
        pub = n.advertise<nav_msgs::OccupancyGrid>("gfgrid", 1000);
        while (n.ok())
        {
            load("zappos.json");
            std::cout << "Success\n";
        }
    }
    catch (std::exception &e)
    {
        std::cout << "Error: " << e.what() << "\n";
    }
    return 0;
}
