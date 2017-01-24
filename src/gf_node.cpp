#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"

#include "geodesy/wgs84.h"





#include "jsoncpp/json/json.h"
#include "jsoncpp/json/writer.h"
#include "jsoncpp/json/reader.h"
#include "geofrenzy/entitlement.h"
#include "geofrenzy/entitlement_service.h"
#include "geofrenzy/entitlement_list_service.h"
#include "swri_transform_util/transform_util.h"





extern "C" char *ambient_fences_geojson_zoom(double lng, double lat, int lvl, int myclass);
extern "C" char *ambient_fences_geojson_roi(double lng, double lat, int lvl, int myclass);
extern "C" char *class_entitlements_properties_json(int myclass);


class fence_server
{
    public:
        void callback(const sensor_msgs::NavSatFix::ConstPtr &msg);
        fence_server(uint64_t fence_class,ros::NodeHandle n);
    private:
        uint64_t fence_class;
        double_t previous_lat;
        double_t previous_long;
        ros::NodeHandle n;


};

fence_server::fence_server(uint64_t fence_class, ros::NodeHandle n)
{
    fence_server::fence_class = fence_class;
    fence_server::n = n;
};

void fence_server::callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    std::cout << "start fence_server Callback\n";
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
        std::string message;
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
        char *td = ambient_fences_geojson_roi(msg->longitude, msg->latitude, 16, fence_class);
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
        std::cout << "Failed to parse fences\n"
                  << reader.getFormattedErrorMessages();
        return;
    }
};


class list_metadata
{
        /**
        * This class holds the list of entitlements for a fence class
        */
    public:

        list_metadata();
        bool append_entitlement(uint64_t entitlement);
        bool listCallback(geofrenzy::entitlement_list_service::Request &req,
                          geofrenzy::entitlement_list_service::Response &res);
    private:
        geofrenzy::entitlement_list_service::Response list_resp_;
        std::vector<uint64_t> list_vec;
};

bool list_metadata::listCallback(geofrenzy::entitlement_list_service::Request &req,
                                 geofrenzy::entitlement_list_service::Response &res)
{
    ROS_INFO("Requesting entitlement list");
    // request is empty; we ignore it

    // = operator is overloaded to make deep copy (tricky!)
    list_resp_.entitlement = list_vec;
    res = list_resp_;
    ROS_INFO("Sending entitlement list");

    return true;
};

bool list_metadata::append_entitlement(uint64_t entitlement)
{

    list_vec.push_back(entitlement);
    return true;
};

list_metadata::list_metadata()
{

};

class entitlement_metadata
{
        /**
            * This class holds the entitlement metadata for a fence class
            */
    public:

        entitlement_metadata(uint64_t entitlement, std::string ent_base);
        bool entitlementCallback(geofrenzy::entitlement_service::Request &req,
                                 geofrenzy::entitlement_service::Response &res);
    private:
        geofrenzy::entitlement_service::Response ent_resp_;
};



entitlement_metadata::entitlement_metadata(uint64_t entitlement, std::string ent_base)
{

    //ent_resp_.msg.header.frame_id = "entitlement";
    //ent_resp_.msg.header.stamp = ros::Time::now();
        std::cout << entitlement << "\n";
    std::cout << ent_base << "\n";
    ent_resp_.entitlement = entitlement;
    ent_resp_.ent_base = ent_base;

};

/** Callback invoked when someone requests our service */
bool entitlement_metadata::entitlementCallback(geofrenzy::entitlement_service::Request &req,
        geofrenzy::entitlement_service::Response &res)
{
    ROS_INFO("Requesting entitlement");
        std::cout << ent_resp_.entitlement << "\n";
    std::cout << ent_resp_.ent_base << "\n";
    // request is empty; we ignore it

    // = operator is overloaded to make deep copy (tricky!)
    //res = ent_resp_;
    res.entitlement = ent_resp_.entitlement;
    res.ent_base = ent_resp_.ent_base;
    ROS_INFO("Sending entitlement");

    return true;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "gf_node", ros::init_options::AnonymousName);

    std::string myclass_idx_str = argv[1];
    std::stringstream convert(myclass_idx_str);
    uint64_t myclass_idx;
    convert >> myclass_idx;


    //gf_fence_request gpsfix;
	ros::NodeHandle n;
    ros::Subscriber gps;
    Json::Value entitlements_root;
    Json::Reader reader;

    std::vector<ros::ServiceServer> service_vec;
    list_metadata ent_list;

    try
    {

        char *t;
        char *td = class_entitlements_properties_json(myclass_idx);
        t =  td ;
        std::string   st(t);
        st = "{" + st + "}";
        bool parsingSuccessful = reader.parse( st, entitlements_root );
        if ( !parsingSuccessful )
        {
            // report to the user the failure and their locations in the document.
            std::cout  << "Failed to parse configuration\n"
                       << reader.getFormattedErrorMessages();
            return 1;
        };


        int class_idx = entitlements_root["class_metadata"]["class_idx"].asInt();
        Json::Value entitlements = entitlements_root["class_metadata"]["entitlements"];
        for (int i = 0; i < entitlements.size(); i++) {
            uint64_t ent_idx_int = entitlements[i]["ent_idx"].asInt();
            std::ostringstream class_idx_ss;
            class_idx_ss << class_idx;
            std::string class_idx_str = class_idx_ss.str();
            std::ostringstream ent_idx_ss;
            ent_idx_ss << ent_idx_int;
            std::string ent_idx_str = ent_idx_ss.str();
            std::string ent_base_str = entitlements[i]["ent_base"].asString();
            
            std::cout << "for class_idx=";
            std::cout << class_idx;
            std::cout << " ent_idx of ";
            std::cout << ent_idx_int;
            std::cout << " is of ent_base type ";
            std::cout <<  ent_base_str;
            std::cout << "\n";
            std::cout.flush();

            entitlement_metadata myentitlement(ent_idx_int,ent_base_str);
            std::string mypath = "geofrenzy/" +class_idx_str + "/" + ent_idx_str;
            service_vec.push_back(n.advertiseService(mypath,&entitlement_metadata::entitlementCallback, &myentitlement));
            ent_list.append_entitlement(ent_idx_int);
        }
      //  fence_server fs(class_idx,n);
        std::string mycpath = "geofrenzy/" + myclass_idx_str + "/list";
        service_vec.push_back(n.advertiseService(mycpath,&list_metadata::listCallback, &ent_list));
      //  gps = n.subscribe("fix", 1, &fence_server::callback, &fs);


        ros::spin();
        //MapServer ms;
        //ros::ServiceServer service;
        //gps = n.subscribe("fix", 1, &MapServer::mapServerCallback, &ms);
        //printf("gps subscribed\n");
        //service = n.advertiseService("static_map", &MapServer::mapCallback, &ms);
        //printf("static_map service advertised\n");
        //ros::spin();
    }
    catch (std::exception &e)
    {
        std::cout << "Error: " << e.what() << "\n";
    }
    return 0;
}


