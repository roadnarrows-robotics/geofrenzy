//
// System
//
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

//
// ROS
//
#include "ros/ros.h"
#include "ros/console.h"

//
// ROS generated core messages
//
#include "std_msgs/String.h"
#include "nav_msgs/MapMetaData.h"

//
// ROS generated Geofrenzy messages
//
#include "geofrenzy/GfDistFeatureCollection.h"
#include "geofrenzy/GfGeoFeatureCollection.h"
#include "geofrenzy/GfDwellBoolset.h"
#include "geofrenzy/GfDwellColor.h"
#include "geofrenzy/GfDwellProfile.h"
#include "geofrenzy/GfDwellThreshold.h"

#include "geofrenzy/entitlement.h" // RDK deprecated


//
// ROS generated Geofrenzy services
//

#include "jsoncpp/json/json.h"
#include "jsoncpp/json/writer.h"
#include "jsoncpp/json/reader.h"

//
// Geofrenzy
//
#include "geodesy/wgs84.h"

#include "geofrenzy/entitlement_service.h"
#include "geofrenzy/entitlement_list_service.h"

#include "swri_transform_util/transform_util.h"


extern "C" 
{
  char *ambient_fences_geojson_zoom(double lng,
                                    double lat,
                                    int lvl,
                                    int myclass);

  char *ambient_fences_geojson_roi(double lng,
                                   double lat,
                                   int lvl,
                                   int myclass);

  char *class_entitlements_properties_json(int myclass);
}

namespace geofrenzy
{
  /**
   * this class implements the GeoJson Fence Server
   */
  class FenceServer
  {
    public:

      /*! map of ROS server services type */
      typedef std::map<std::string, ros::ServiceServer> MapServices;

      /*! map of ROS client services type */
      typedef std::map<std::string, ros::ServiceClient> MapClientServices;
    
      /*! map of ROS publishers type */
      typedef std::map<std::string, ros::Publisher> MapPublishers;

      /*! map of ROS subscriptions type */
      typedef std::map<std::string, ros::Subscriber> MapSubscriptions;

      FenceServer(uint64_t new_class, ros::NodeHandle newnh, double hz) :
          m_fence_class(new_class), m_nh(newnh), m_hz(hz)
      {
        /**
         * this metthod instantiates a fence server
         * \param[in] new_class class index for the node
         * \param[in] newnh ros node handle for the node
         */
        ROS_DEBUG_STREAM("fence_class alloc = " << m_fence_class);

        //RDK std::stringstream fence_class_stream;
        //RDK fence_class_stream << m_fence_class;
        //RDK std::string fc_string = "geofrenzy/" + 
        //RDK                           fence_class_stream.str() +
        //RDK                           "/featureCollection/json";
        //RDK metadata_pub = newnh.advertise<std_msgs::String>(fc_string, 1, true);
        m_nPubRepeatCnt = 0;
      };
  
      bool init()
      {
#if TBD // RDK
        Json::Value   entitlements_root;
        Json::Reader  reader;

        char *t;
        char *td = class_entitlements_properties_json(m_fence_class);
        t = td;

        std::string st(t);
        st = "{" + st + "}";

        bool parsingSuccessful = reader.parse(st, entitlements_root);

        if (!parsingSuccessful)
        {
          // report to the user the failure and their locations in the document.
          ROS_ERROR_STREAM(m_nh.getNamespace()
              << ": Failed to parse configuration\n"
              << reader.getFormattedErrorMessages());
          return false;
        };
#endif // TBD RDK

        return true;
      }

      /*!
       * \brief Advertise all server services.
       */
      void advertiseServices()
      {
      }

      /*!
       * \brief Initialize client services.
       */
      void clientServices()
      {
        // No client services
      }

      /*!
       * \brief Advertise all publishers.
       *
       * \param nQueueDepth   Maximum queue depth.
       */
      void advertisePublishers(int nQueueDepth=5)
      {
        std::stringstream ss;
        std::string       strPub;

        ss << "geofrenzy/" << m_fence_class << "/featureCollection/";

        strPub = ss.str() + "json";
        m_publishers[strPub] =
          m_nh.advertise<std_msgs::String>(strPub, 1, true);

        strPub = ss.str() + "geo";
        m_publishers[strPub] =
          m_nh.advertise<geofrenzy::GfGeoFeatureCollection>(strPub, 1, true);

        strPub = ss.str() + "distance";
        m_publishers[strPub] =
          m_nh.advertise<geofrenzy::GfDistFeatureCollection>(strPub, 1, true);
      }

      /*!
       * \brief Subscribe to all topics.
       *
       * \param nQueueDepth   Maximum queue depth.
       */
      void subscribeToTopics(int nQueueDepth=5)
      {
        std::string   strSub;

        // fix location
        strSub = "fix";
        m_subscriptions[strSub] = m_nh.subscribe(strSub, 1,
                                          &FenceServer::cbNavSatFix,
                                          &(*this));
      }

      /*!
       * \brief Publish.
       *
       * Call in main loop.
       */
      virtual void publish()
      {
        if( m_nPubRepeatCnt > 0 )
        {
          std::stringstream ss;
          std::string       strPub;

          ss << "geofrenzy/" << m_fence_class << "/featureCollection/";

          strPub = ss.str() + "json";
          m_publishers[strPub].publish(m_msgFcJson);

          --m_nPubRepeatCnt;
        }
      }

      void AppendEntitlement(uint64_t newentitlement)
      {
        /**
         * this metthod adds a new entitlement index to the list of Entitlements
         * for creating node for publication
         * \param[in] newentitlement new entitlement idx
         */
        std::stringstream fence_class_stream;
        fence_class_stream << m_fence_class;
        std::stringstream newentitlement_stream;
        newentitlement_stream << newentitlement;
        std::string ad_string = "geofrenzy/" +
                                  fence_class_stream.str() +
                                  "/" + newentitlement_stream.str() +
                                  "/dwell/json";
        entitlement_map[newentitlement] = m_nh.advertise<std_msgs::String>(ad_string,
            1, true);
      }
  
      void cbNavSatFix(const sensor_msgs::NavSatFix::ConstPtr &msg)
      {
        const char    *topic = "fix";

        ROS_DEBUG_STREAM(m_nh.getNamespace() << "/" << topic);

        Json::Value   root; // will contains the root value after parsing.
        Json::Reader  reader;

        ROS_INFO_STREAM("Previous lat,long: "
            << m_previous_lat
            << ", "
            << m_previous_long);

        // no gps acquired
        if (msg->status.status == -1)
        {
          ROS_INFO_STREAM("No GPS acquired.");
          return;
        }
  
        ROS_INFO_STREAM("New lat,long: "
            << msg->latitude
            << ", "
            << msg->longitude);
  
        double distance;
  
        distance = swri_transform_util::GreatCircleDistance(
                              msg->latitude, msg->longitude,
                              m_previous_lat, m_previous_long);

        ROS_DEBUG_STREAM("Distance = " << distance);
  
        // ignore this for now, testing
        if (distance < 0)
        {
          return;
        }
        else
        {
          m_previous_lat  = msg->latitude;
          m_previous_long = msg->longitude;
        }
  
        ROS_DEBUG_STREAM("Ready to check params.");

        std::string filename;
        char *t;
  
        if (m_nh.getParam("geojson_file", filename))
        {
          ROS_DEBUG_STREAM("Reading file " << filename);

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
          ROS_DEBUG_STREAM("Done reading file.");
        }

        else
        {
          ROS_DEBUG_STREAM("Checking fence roi for class " << m_fence_class);

          ROS_DEBUG_STREAM("Calling ambient_fences_geojson_roi().");

          char *td = ambient_fences_geojson_roi(msg->longitude, msg->latitude,
                                                6, m_fence_class);

          ROS_DEBUG_STREAM("Done.");
          t = td;
        }
  
        ROS_DEBUG_STREAM("begin\n" << t << "end");

        std::stringstream t_stream;
        t_stream << t;

        updateFeatureCollectionMsgs(t);
        //RDK std_msgs::String fence_message;
        //RDK fence_message.data = t_stream.str();
        //RDK metadata_pub.publish(fence_message);
  
        bool parsingSuccessful = reader.parse(t, root);
        if (!parsingSuccessful)
        {
          // report to the user the failure and their locations in the document.
          ROS_ERROR_STREAM("Failed to parse fences.\n"
                                << reader.getFormattedErrorMessages());
          return;
        }
  
        std::map <uint64_t, Json::Value> entitlement_temp_map;
        std::map<uint64_t, bool> inorout;
        Json::Value FeatureList = root["features"];

        for (Json::Value::iterator feature = FeatureList.begin();
            feature != FeatureList.end();
            ++feature)
        {
          Json::Value Entitlements = (*feature)["properties"]["class_metadata"]["entitlements"];
          for (Json::Value::iterator entitlement = Entitlements.begin();
               entitlement != Entitlements.end();
               ++entitlement)
          {
            uint64_t ent_idx = (*entitlement)["ent_idx"].asInt();
            entitlement_temp_map[ent_idx] = *entitlement;
            Json::FastWriter fastWriter;

            std::string entitlement_string = fastWriter.write(*entitlement);

            Json::Value Inout = (*feature)["properties"]["inout"];

            ROS_DEBUG_STREAM("entitlement start\n"
                << entitlement_string << "\n"
                << ent_idx << "\n"
                << "entitlement end");

            ROS_DEBUG_STREAM("inout = " << Inout);
  
            if (Inout.asString().compare("i") == 0)
            {
              inorout[ent_idx] = inorout[ent_idx] | true;
            }
            else
            {
              inorout[ent_idx] = inorout[ent_idx] | false;
            }
          }
  
          for (std::map<uint64_t, Json::Value>::iterator val_it = entitlement_temp_map.begin();
              val_it != entitlement_temp_map.end();
              ++val_it)
          {
            uint64_t tempidx = val_it->first;
            val_it->second["dwell"] = inorout[tempidx];
            std::stringstream fence_class_str;
            fence_class_str << m_fence_class;
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


    protected:
      /*!
       * \brief Update feature collection messages from current location.
       */
      void updateFeatureCollectionMsgs(std::string strJson)
      {
        m_msgFcJson.data = strJson;

        ++m_nPubRepeatCnt;
      }

      /*!
       * \brief Fill in ROS standard message header.
       *
       * \param [out] header    Message header.
       * \param nSeqNum         Sequence number.
       * \param strFrameId      Frame id. No frame = "0", global frame = "1".
       */
      void stampHeader(std_msgs::Header  &header,
                       int32_t            nSeqNum = 0,
                       const std::string &strFrameId = "geofrenzy")
      {
        header.seq      = nSeqNum;
        header.stamp    = ros::Time::now();
        header.frame_id = strFrameId;
      }

    private:
      ros::NodeHandle m_nh;
      double          m_hz;
      uint64_t        m_fence_class;

      // ROS services, publishers, subscriptions.
      MapServices       m_services;       ///< Laelaps control server services
      MapClientServices m_clientServices; ///< Laelaps control client services
      MapPublishers     m_publishers;     ///< Laelaps control publishers
      MapSubscriptions  m_subscriptions;  ///< Laelaps control subscriptions

      double_t m_previous_lat;
      double_t m_previous_long;

      // Message for publishing
      std_msgs::String        m_msgFcJson;
      GfGeoFeatureCollection  m_msgFcGeo;
      GfDistFeatureCollection m_msgFcDist;
      int                     m_nPubRepeatCnt;

      //RDK ros::Publisher metadata_pub;
      std::map <uint64_t, ros::Publisher> entitlement_map;
  
  }; // class FenceServer
  
  /**
   * This class holds the list of entitlements for a fence class
   */
  class ListMetadata
  {
    private:
      geofrenzy::entitlement_list_service::Response list_resp_;
      std::vector <uint64_t> list_vec;
  
    public:
  
      // ListMetadata();
      // bool append_entitlement(uint64_t entitlement);
      //  bool listCallback(geofrenzy::entitlement_list_service::Request &req,
      //                    geofrenzy::entitlement_list_service::Response &res);
  
  
  
      bool listCallback(geofrenzy::entitlement_list_service::Request &req,
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
  
  
      bool append_entitlement(uint64_t entitlement)
      {
        list_vec.push_back(entitlement);
        return true;
      };
  
      ListMetadata()
      {
      };
  
      ~ListMetadata()
      {
        std::cout << "Destroyed ListMetadata";
      };
  }; // class ListMetadata
  
  /**
   * This class holds the entitlement metadata for a fence class
   */
  class EntitlementMetadata
  {
    private:
      geofrenzy::entitlement_service::Response ent_resp_;
  
    public:
  
      EntitlementMetadata(uint64_t entitlement, std::string ent_base)
      {
        //ent_resp_.msg.header.frame_id = "entitlement";
        //ent_resp_.msg.header.stamp = ros::Time::now();
        std::cout << entitlement << "\n";
        std::cout << ent_base << "\n";
        ent_resp_.entitlement = entitlement;
        ent_resp_.ent_base = ent_base;
      };
  
      /** Callback invoked when someone requests our service */
      bool entitlementCallback(geofrenzy::entitlement_service::Request &req,
                               geofrenzy::entitlement_service::Response &res)
      {
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
  
      ~EntitlementMetadata()
      {
        std::cout << "destroying entitlementmetadata\n";
      };
  
  }; // class EntitlementMetadata

} // namespace geofrenzy

/**
 * This node queries the fence delivery network and produces topics
 * of the form /geofrenzy/[class idx]/[entitlement index]/dwell/json
 * and /geofrenzy/[class index]/featureCollection/json
 * representing the occupancy of a fence with a class and entitlement value
 * and the Geojson of given fences within a class repectively
 * \param class class index for the node
 */
int main(int argc, char **argv)
{
  double hz = 10.0;

  //
  // Geofrenzy specific command-line parsing.
  // RDK integrate with ros command-line parsing
  //
  std::string myclass_idx_str = argv[1];
  std::string node_name = "gf_node_" + myclass_idx_str;

  std::stringstream convert(myclass_idx_str);
  uint64_t myclass_idx;
  convert >> myclass_idx;

  // 
  // Initialize the node. Parse the command line arguments and environment to
  // determine ROS options such as node name, namespace and remappings.
  // This call does not contact the master. This lets you use
  // ros::master::check() and other ROS functions after calling ros::init()
  // to check on the status of the master.
  //
  ros::init(argc, argv, node_name);

  // actual ROS-given node name
  node_name = ros::this_node::getName();

  //
  // A ctrl-c interrupt will stop attempts to connect to the ROS core.
  //
  ros::NodeHandle nh(node_name);

  //
  // Failed to connect.
  //
  if( !ros::master::check() )
  {
    // add optional non-ROS unit tests here, then simply exit.
    return 0;
  }

  ROS_INFO_STREAM(node_name << ": Node started.");

  //
  // Create fence server node.
  //
  geofrenzy::FenceServer fs(myclass_idx, nh, hz);

  //
  // Initialize fence server.
  //
  if( !fs.init() )
  {
    ROS_FATAL_STREAM(node_name << ": Failed to initialize.");
    return 2;
  }

  ROS_INFO_STREAM(node_name << ": Node initialized.");

  //
  // Advertise services.
  //
  fs.advertiseServices();

  ROS_INFO_STREAM(node_name << ": Advertised services registered.");

  //
  // Advertise publishers.
  //
  fs.advertisePublishers();
  
  ROS_INFO_STREAM(node_name << ": Advertised publishers registered.");
  
  //
  // Subscribed to topics.
  //
  fs.subscribeToTopics();
  
  ROS_INFO_STREAM(node_name << ": Subscribed topics registered.");

  // set loop rate in Hertz
  ros::Rate loop_rate(hz);

  ROS_INFO_STREAM(node_name << "Ready.");

  //gf_fence_request gpsfix;
  //RDKros::NodeHandle nh;

  ros::Subscriber gps;

  Json::Value entitlements_root;
  Json::Reader reader;

  std::vector < ros::ServiceServer * > service_vec;
  geofrenzy::ListMetadata ent_list;

  try
  {
    char *t;
    char *td = class_entitlements_properties_json(myclass_idx);
    t = td;
    std::string st(t);
    st = "{" + st + "}";

    bool parsingSuccessful = reader.parse(st, entitlements_root);

    if (!parsingSuccessful)
    {
      // report to the user the failure and their locations in the document.
      std::cout << "Failed to parse configuration\n"
                              << reader.getFormattedErrorMessages();
      return 1;
    };


    int class_idx = entitlements_root["class_metadata"]["class_idx"].asInt();
    Json::Value entitlements = entitlements_root["class_metadata"]["entitlements"];

    for (int i = 0; i < entitlements.size(); i++)
    {
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

      geofrenzy::EntitlementMetadata *myentitlement =
        new geofrenzy::EntitlementMetadata(ent_idx_int, ent_base_str);
      // *myentitlement = EntitlementMetadata(ent_idx_int,ent_base_str);
      std::string mypath = "geofrenzy/" + class_idx_str + "/" + ent_idx_str;

      ros::ServiceServer *eservice = new ros::ServiceServer;
      *eservice = nh.advertiseService(mypath,
          &geofrenzy::EntitlementMetadata::entitlementCallback, myentitlement);
      service_vec.push_back(eservice);
      ent_list.append_entitlement(ent_idx_int);
      fs.AppendEntitlement(ent_idx_int);
    }

    std::string mycpath = "geofrenzy/" + myclass_idx_str + "/list";
    ros::ServiceServer cservice = nh.advertiseService(mycpath,
        &geofrenzy::ListMetadata::listCallback, &ent_list);

    //RDK gps = nh.subscribe("fix", 1, &geofrenzy::FenceServer::cbNavSatFix, &fs);

    //ros::spin();

    //MapServer ms;
    //ros::ServiceServer service;
    //gps = nh.subscribe("fix", 1, &MapServer::mapServerCallback, &ms);
    //printf("gps subscribed\n");
    //service = nh.advertiseService("static_map", &MapServer::mapCallback, &ms);
    //printf("static_map service advertised\n");
    //ros::spin();
  }

  catch (std::exception &e)
  {
    std::cout << "Error: " << e.what() << "\n";
  }

  //
  // Main loop.
  //
  while( ros::ok() )
  {
    // make any callbacks on pending ROS events
    ros::spinOnce(); 

    // publish all advertized topics
    fs.publish();

    // sleep to keep at loop rate
    loop_rate.sleep();
  }

  return 0;
}
