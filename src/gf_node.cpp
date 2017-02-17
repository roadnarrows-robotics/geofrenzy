#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "nav_msgs/MapMetaData.h"

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


class FenceServer {
        /**
        * this class implements the GeoJson Fence Server
        */

    private:
        uint64_t fence_class;
        double_t previous_lat;
        double_t previous_long;
        ros::NodeHandle n;
        ros::Publisher metadata_pub;
        std::map <uint64_t, ros::Publisher> entitlement_map;

    public:



        FenceServer(uint64_t new_class, ros::NodeHandle newnh) {
            /**
            * this metthod instantiates a fence server
            * \param[in] new_class class index for the node
            * \param[in] newnh ros node handle for the node
            */
            fence_class = new_class;
            n = newnh;
            std::cout << "fence_class alloc=" << fence_class << "\n";
            std::cout.flush();
            std::stringstream fence_class_stream;
            fence_class_stream << fence_class;
            std::string fc_string = "geofrenzy/" + fence_class_stream.str() + "/featureCollection/json";
            metadata_pub = newnh.advertise<std_msgs::String>(fc_string, 1, true);
        };

        void AppendEntitlement(uint64_t newentitlement) {
            /**
            * this metthod adds a new entitlement index to the list of Entitlements
            * for creating node for publication
            * \param[in] newentitlement new entitlement idx
            */
            std::stringstream fence_class_stream;
            fence_class_stream << fence_class;
            std::stringstream newentitlement_stream;
            newentitlement_stream << newentitlement;
            std::string ad_string =
                "geofrenzy/" + fence_class_stream.str() + "/" + newentitlement_stream.str() + "/dwell/json";
            entitlement_map[newentitlement] = n.advertise<std_msgs::String>(ad_string, 1, true);
        }


        void callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
            std::cout << "start FenceServer Callback\n";
            Json::Value root; // will contains the root value after parsing.
            Json::Reader reader;
            std::cout << previous_lat;
            std::cout << previous_long;
            std::cout << "*******************************************************************latlong\n";
            std::cout << msg->latitude;
            std::cout << msg->longitude;

            if (msg->status.status == -1) {
                std::cout << "no gps acquired\n";
                // no gps acquired
                return;
            }

            double distance;

            distance = swri_transform_util::GreatCircleDistance(msg->latitude, msg->longitude, previous_lat, previous_long);
            std::cout << "distance=" << distance << "\n";
            std::cout.flush();

            if (distance < 1) {
                return;
            } else {
                previous_lat = msg->latitude;
                previous_long = msg->longitude;
            }
            std::string filename;
            char *t;
            std::cout << "ready to check params\n";
            std::cout.flush();
            if (n.getParam("geojson_file", filename)) {
                //char *buf;
                std::cout << "read file \n";
                std::cout.flush();
                std::ifstream in(filename.c_str());
                std::string message;
                while (in) {
                    message.push_back(in.get());
				}
                    char *bt = &message[0u];
                    t = bt;
                    //std::vector<char> buf(message.c_str(), message.c_str() + message.size() + 1);
                    //t = buf;
                    std::cout << "done read file \n";
                    std::cout.flush();
                }
                else {
                    std::cout << "ready to check fence roi\n";
                    std::cout << "longitude=" << msg->longitude << "\n";
                    std::cout << "latitude=" << msg->latitude << "\n";
                    std::cout << "fence_class=" << fence_class << "\n";
                    std::cout.flush();
                    char *td = ambient_fences_geojson_roi(msg->longitude, msg->latitude, 6, fence_class);
                    t = td;
                }
                std::cout << "begin *t\n";
                std::cout << t;
                std::cout << "end *t\n";
                std::cout.flush();
                std::stringstream t_stream;
                t_stream << t;
                std_msgs::String fence_message;
                fence_message.data = t_stream.str();
                metadata_pub.publish(fence_message);

                bool parsingSuccessful = reader.parse(t, root);
                if (!parsingSuccessful) {
                    // report to the user the failure and their locations in the document.
                    std::cout << "Failed to parse fences\n"
                              << reader.getFormattedErrorMessages();
                    return;
                }
                std::map <uint64_t, Json::Value> entitlement_temp_map;
                std::map<uint64_t, bool> inorout;
                Json::Value FeatureList = root["features"];
                for (Json::Value::iterator feature = FeatureList.begin(); feature != FeatureList.end(); ++feature) {
                    Json::Value Entitlements = (*feature)["properties"]["class_metadata"]["entitlements"];
                    for (Json::Value::iterator entitlement = Entitlements.begin();
                            entitlement != Entitlements.end(); ++entitlement) {
                        uint64_t ent_idx = (*entitlement)["ent_idx"].asInt();
                        entitlement_temp_map[ent_idx] = *entitlement;
                        Json::FastWriter fastWriter;
                        std::string entitlement_string = fastWriter.write(*entitlement);
                        std::cout << "entitlement start\n";
                        std::cout << entitlement_string << "\n";
                        std::cout << ent_idx << "\n";
                        std::cout << "entitlement end\n";
                        Json::Value Inout = (*feature)["properties"]["inout"];
                        std::cout << Inout << "\n";

                        if (Inout.asString().compare("i") == 0) {
                            std::cout << "+++++TRUE+++++\n";
                            inorout[ent_idx] = inorout[ent_idx] | true;
                        } else {
                            inorout[ent_idx] = inorout[ent_idx] | false;
                            std::cout << "-----FALSE------\n";
                        }
                    }
                    for (std::map<uint64_t, Json::Value>::iterator val_it = entitlement_temp_map.begin();
                            val_it != entitlement_temp_map.end(); ++val_it) {
                        uint64_t tempidx = val_it->first;
                        val_it->second["dwell"] = inorout[tempidx];
                        std::stringstream fence_class_str;
                        fence_class_str << fence_class;
                        val_it->second["class_idx"] = fence_class_str.str();
                        Json::FastWriter fastWriter;
                        std::string entitlement_temp_string = fastWriter.write(val_it->second);
                        std_msgs::String entitlement_message;
                        entitlement_message.data = entitlement_temp_string;
                        //entitlement_message.header.stamp = ros::Time::now();
                        entitlement_map[tempidx].publish(entitlement_message);
                        // dwell_pub.publish(entitlement_message);
                    }

                };
            };
        };

        class ListMetadata {
                /**
                * This class holds the list of entitlements for a fence class
                */
            private:
                geofrenzy::entitlement_list_service::Response list_resp_;
                std::vector <uint64_t> list_vec;

            public:

                // ListMetadata();
                // bool append_entitlement(uint64_t entitlement);
                //  bool listCallback(geofrenzy::entitlement_list_service::Request &req,
                //                    geofrenzy::entitlement_list_service::Response &res);



                bool listCallback(geofrenzy::entitlement_list_service::Request &req,
                                  geofrenzy::entitlement_list_service::Response &res) {
                    ROS_INFO("Requesting entitlement list");
                    // request is empty; we ignore it

                    // = operator is overloaded to make deep copy (tricky!)
                    list_resp_.entitlement = list_vec;
                    res = list_resp_;
                    ROS_INFO("Sending entitlement list");

                    return true;
                };


                bool append_entitlement(uint64_t entitlement) {

                    list_vec.push_back(entitlement);
                    return true;
                };

                ListMetadata() {

                };

                ~ListMetadata() {
                    std::cout << "Destroyed ListMetadata";
                };
        };

        class EntitlementMetadata {
                /**
                    * This class holds the entitlement metadata for a fence class
                    */

            private:
                geofrenzy::entitlement_service::Response ent_resp_;
            public:


                EntitlementMetadata(uint64_t entitlement, std::string ent_base) {

                    //ent_resp_.msg.header.frame_id = "entitlement";
                    //ent_resp_.msg.header.stamp = ros::Time::now();
                    std::cout << entitlement << "\n";
                    std::cout << ent_base << "\n";
                    ent_resp_.entitlement = entitlement;
                    ent_resp_.ent_base = ent_base;

                };

                /** Callback invoked when someone requests our service */
                bool entitlementCallback(geofrenzy::entitlement_service::Request &req,
                                         geofrenzy::entitlement_service::Response &res) {
                    ROS_INFO("Requesting entitlement");
                    std::cout << ent_resp_.entitlement << "\n";
                    //std::cout << ent_resp_.ent_base << "\n";
                    // request is empty; we ignore it

                    // = operator is overloaded to make deep copy (tricky!)
                    //res = ent_resp_;
                    res.entitlement = ent_resp_.entitlement;
                    res.ent_base = ent_resp_.ent_base;
                    ROS_INFO("Sending entitlement");

                    return true;
                };

                ~EntitlementMetadata() {
                    std::cout << "destroying entitlementmetadata\n";
                };

        };

        /**
         * This node queries the fence delivery network and produces topics
         * of the form /geofrenzy/[class idx]/[entitlement index]/dwell/json
         * and /geofrenzy/[class index]/featureCollection/json
         * representing the occupancy of a fence with a class and entitlement value
         * and the Geojson of given fences within a class repectively
         * \param class class index for the node
         */


        int main(int argc, char **argv) {

            std::string myclass_idx_str = argv[1];
            std::string node_name = "gf_node_" + myclass_idx_str;
            ros::init(argc, argv, node_name);


            std::stringstream convert(myclass_idx_str);
            uint64_t myclass_idx;
            convert >> myclass_idx;


            //gf_fence_request gpsfix;
            ros::NodeHandle n;
            ros::Subscriber gps;
            Json::Value entitlements_root;
            Json::Reader reader;

            std::vector < ros::ServiceServer * > service_vec;
            ListMetadata ent_list;
            FenceServer fs(myclass_idx, n);

            try {

                char *t;
                char *td = class_entitlements_properties_json(myclass_idx);
                t = td;
                std::string st(t);
                st = "{" + st + "}";
                bool parsingSuccessful = reader.parse(st, entitlements_root);
                if (!parsingSuccessful) {
                    // report to the user the failure and their locations in the document.
                    std::cout << "Failed to parse configuration\n"
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
                    std::cout << ent_base_str;
                    std::cout << "\n";
                    std::cout.flush();

                    EntitlementMetadata *myentitlement = new EntitlementMetadata(ent_idx_int, ent_base_str);
                    // *myentitlement = EntitlementMetadata(ent_idx_int,ent_base_str);
                    std::string mypath = "geofrenzy/" + class_idx_str + "/" + ent_idx_str;

                    ros::ServiceServer *eservice = new ros::ServiceServer;
                    *eservice = n.advertiseService(mypath, &EntitlementMetadata::entitlementCallback, myentitlement);
                    service_vec.push_back(eservice);
                    ent_list.append_entitlement(ent_idx_int);
                    fs.AppendEntitlement(ent_idx_int);
                }

                std::string mycpath = "geofrenzy/" + myclass_idx_str + "/list";
                ros::ServiceServer cservice = n.advertiseService(mycpath, &ListMetadata::listCallback, &ent_list);


                gps = n.subscribe("fix", 1, &FenceServer::callback, &fs);


                ros::spin();
                //MapServer ms;
                //ros::ServiceServer service;
                //gps = n.subscribe("fix", 1, &MapServer::mapServerCallback, &ms);
                //printf("gps subscribed\n");
                //service = n.advertiseService("static_map", &MapServer::mapCallback, &ms);
                //printf("static_map service advertised\n");
                //ros::spin();
            }
            catch (std::exception &e) {
                std::cout << "Error: " << e.what() << "\n";
            }
            return 0;
        }


