////////////////////////////////////////////////////////////////////////////////
//
// Package:   Geofrenzy Robot ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/geofrenzy
//
// File:      gf_server.cpp
//
/*! \file
 *
 * \brief The Geofrenzy ROS node source bridge server between the Geofrenzy
 * Portal and ROS.
 *
 * \author Bill Coon (bill@roadnarrows.com)
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Maintainer:
 * Chris Stradtman (chris.stradtman@geo.network)
 *
 * \par Copyright:
 * (C) 2017  GeoNetwork
 * (http://www.geo.network)
 * \n All Rights Reserved
 *
 * \par License
 * Apache 2.0
 * 
 * EULA:
 * See EULA.md
 */
////////////////////////////////////////////////////////////////////////////////

//
// System
//
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>

//
// Add-on SDKs
//
#include "jsoncpp/json/json.h"
#include "jsoncpp/json/writer.h"
#include "jsoncpp/json/reader.h"
#include "geodesy/wgs84.h"
#include "swri_transform_util/transform_util.h"
#include "boost/bind.hpp"

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
#include "geofrenzy/GfJsonFeatureCollection.h"
#include "geofrenzy/GfDwellBoolset.h"
#include "geofrenzy/GfDwellColor.h"
#include "geofrenzy/GfDwellJson.h"
#include "geofrenzy/GfDwellProfile.h"
#include "geofrenzy/GfDwellThreshold.h"
#include "geofrenzy/StringStamped.h"

//
// ROS generated Geofrenzy services
//
#include "geofrenzy/GetEntitlement.h"
#include "geofrenzy/GetEntitlementList.h"

//
// Geofrenzy
//

// Note: should be in a geofrenzy header file
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

#include "gf_types.h"
#include "gf_ros.h"


using namespace geofrenzy::gf_ros;

namespace geofrenzy
{
  //----------------------------------------------------------------------------
  // Geofrenzy Entitlement
  //----------------------------------------------------------------------------
  
  /*!
   * This class holds an entitlement for a Geofrenzy fence class.
   */
  class Entitlement
  {
    public:
      /*!
       * \brief Default initialization constructor.
       *
       * \param gf_class_idx  Geofrenzy class index for this entitlement.
       * \param gf_ent_idx    Geofrenzy entitlement index.
       * \param gf_ent_base   Geofrenzy entitlement base.
       */
      Entitlement(uint64_t          gf_class_idx,
                  uint64_t          gf_ent_idx,
                  const std::string gf_ent_base) :
          m_class_idx(gf_class_idx),
          m_ent_idx(gf_ent_idx),
          m_ent_base(gf_ent_base),
          m_atime(0.0)
      {
        m_eEntDataType  = entBaseToType(gf_ent_base); // entitlement type enum
        m_bDwell        = false;                      // not within geofence
        m_nPublishCnt   = 0;                          // nothing to publish yet

        ROS_DEBUG_STREAM("Constructed Entitlement " << m_ent_idx);
      };
  
      /*!
       * \brief Destructor.
       */
      ~Entitlement()
      {
        ROS_DEBUG_STREAM("Destroyed Entitlement " << m_ent_idx);
      };
  
      /*!
       * \brief Advertise all entitlment services.
       *
       * \param nh    Associated ROS node handle.
       */
      void advertiseServices(ros::NodeHandle &nh)
      {
      }

      /*!
       * \brief Advertise all publishers.
       *
       * \param nh            Associated ROS node handle.
       * \param nQueueDepth   Maximum queue depth.
       */
      void advertisePublishers(ros::NodeHandle &nh, int nQueueDepth=2)
      {
        //
        // Data-specific dwell topic.
        //
        m_strTopicDwellX = makeDwellTopicName(m_ent_idx, m_ent_base);

        switch( m_eEntDataType )
        {
          case GfEntDataTypeBoolset:
            m_publishers[m_strTopicDwellX] =
              nh.advertise<GfDwellBoolset>(m_strTopicDwellX, 1, true);
            break;
          case GfEntDataTypeColor:
            m_publishers[m_strTopicDwellX] =
              nh.advertise<GfDwellColor>(m_strTopicDwellX, 1, true);
            break;
          case GfEntDataTypeProfile:
            m_publishers[m_strTopicDwellX] =
              nh.advertise<GfDwellProfile>(m_strTopicDwellX, 1, true);
            break;
          case GfEntDataTypeThreshold:
            m_publishers[m_strTopicDwellX] =
              nh.advertise<GfDwellThreshold>(m_strTopicDwellX, 1, true);
            break;
          case GfEntDataTypeUndef:
          default:
            ROS_WARN_STREAM("Unknown entitlement base data type = "
              << m_ent_base);
            break;
        }

        //
        // Json encoded string dwell topic.
        //
        m_strTopicDwellJson = makeDwellTopicName(m_ent_idx, "json");

        m_publishers[m_strTopicDwellJson] =
          nh.advertise<GfDwellJson>(m_strTopicDwellJson, 1, true);
      }

      /*!
       * \brief Update entitlement with new information.
       *
       * \param jvEntitlement   Json value of a parsed entitlement.
       * \param bDwell          Current location is [not] within a fence.
       * \param atime           Last Geofrenzy portal access time.
       */
      void update(const Json::Value &jvEntitlement,
                  bool              bDwell,
                  const ros::Time   &atime)
      {
        std::string gf_ent_base = jvEntitlement["ent_base"].asString();

        if( gf_ent_base != m_ent_base )
        {
          ROS_WARN_STREAM("Entitlement base "
              << "'" << gf_ent_base << "'"
              << " not equal expected "
              << "'" << m_ent_base << "'"
              << " - ignoring update.");
          return;
        }

        m_bDwell  = bDwell;
        m_atime   = atime;

        //
        // Populate entitlement specific data.
        //
        switch( m_eEntDataType )
        {
          case GfEntDataTypeBoolset:
            updateDwellBoolsetMsg(jvEntitlement, m_msgDwellBoolset);
            break;
          case GfEntDataTypeColor:
            updateDwellColorMsg(jvEntitlement, m_msgDwellColor);
            break;
          case GfEntDataTypeProfile:
            updateDwellProfileMsg(jvEntitlement, m_msgDwellProfile);
            break;
          case GfEntDataTypeThreshold:
            updateDwellThresholdMsg(jvEntitlement, m_msgDwellThreshold);
            break;
          case GfEntDataTypeUndef:
          default:
            //ROS_WARN_STREAM("Unknown entitlement base data type = "
            //  << m_ent_base);
            break;
        }

        //
        // Populate json data.
        //
        updateDwellJsonMsg(jvEntitlement, m_msgDwellJson);

        ++m_nPublishCnt;
      }

      /*!
       * \brief Get the Geofrenzy class index.
       *
       * \return Index.
       */
      uint64_t classIndex() const
      {
        return m_class_idx;
      }

      /*!
       * \brief Get the Geofrenzy entitlement index.
       *
       * \return Index.
       */
      uint64_t entitlementIndex() const
      {
        return m_ent_idx;
      }

      /*!
       * \brief Get the Geofrenzy entitlement base.
       *
       * \return String.
       */
      const std::string &entitlementBase() const
      {
        return m_ent_base;
      }

      /*!
       * \brief Test if there are publishable messages.
       *
       * \return Returns true or false.
       */
      bool shouldPublish() const
      {
        return m_nPublishCnt > 0;
      }

      /*!
       * \brief Publish all publishable messages.
       */
      void publish()
      {
        if( m_nPublishCnt > 0 )
        {
          //
          // Publish entitlement specific data.
          //
          switch( m_eEntDataType )
          {
            case GfEntDataTypeBoolset:
              m_publishers[m_strTopicDwellX].publish(m_msgDwellBoolset);
              break;
            case GfEntDataTypeColor:
              m_publishers[m_strTopicDwellX].publish(m_msgDwellColor);
              break;
            case GfEntDataTypeProfile:
              m_publishers[m_strTopicDwellX].publish(m_msgDwellProfile);
              break;
            case GfEntDataTypeThreshold:
              m_publishers[m_strTopicDwellX].publish(m_msgDwellThreshold);
              break;
            case GfEntDataTypeUndef:
            default:
              //ROS_WARN_STREAM("Unknown entitlement base data type = "
              //  << m_ent_base);
              break;
          }

          //
          // Publish json encoded string data.
          //
          m_publishers[m_strTopicDwellJson].publish(m_msgDwellJson);

          --m_nPublishCnt;
        }
      }

      /*!
       * \brief Fill the response message with the relevant entitlement data.
       *
       * \param [out] rsp Response message.
       */
      void fill(geofrenzy::GetEntitlement::Response &rsp)
      {
        rsp.ent_header  = m_msgDwellJson.entitlement.ent_header;
        rsp.entitlement = m_msgDwellJson.entitlement.json;
      }

    protected:
      // Geofrenzy entitlement metadata
      uint64_t    m_class_idx;    ///< class index
      uint64_t    m_ent_idx;      ///< entitlement index
      std::string m_ent_base;     ///< entitlement base data type

      // ROS entitlement services and publishers
      MapServices     m_services;       ///< Geofrenzy entitlement services
      MapPublishers   m_publishers;     ///< Geofrenzy entitlement publishers

      // Messaging processing overhead
      GfEntDataType m_eEntDataType;       ///< derived entitlement data type
      std::string   m_strTopicDwellX;     ///< data-specific dwell topic name
      std::string   m_strTopicDwellJson;  ///< json dwell topic name
      ros::Time     m_atime;              ///< last portal access time
      bool          m_bDwell;             ///< [not] within a fence
      int           m_nPublishCnt;        ///< publish counter

      // publishing messages
      GfDwellBoolset    m_msgDwellBoolset;    ///< boolset dwell message
      GfDwellColor      m_msgDwellColor;      ///< color dwell message
      GfDwellProfile    m_msgDwellProfile;    ///< profile dwell message
      GfDwellThreshold  m_msgDwellThreshold;  ///< threshold dwell message
      GfDwellJson       m_msgDwellJson;       ///< json entitlement message

      /*!
       * \brief Update ROS boolset dwell message from Json entitlement value.
       *
       * \param [in]  jv  Json value of parsed entitlement.
       * \param [out] msg ROS data-specific dwell message.
       *
       * \return Returns true on success, false otherwise.
       */
      bool updateDwellBoolsetMsg(const Json::Value &jv,
                                 GfDwellBoolset    &msg)
      {
        updateEntitlementHeader(msg.entitlement.ent_header);

        msg.entitlement.bitset_num = jv["bitset_num"].asUInt();

        stampHeader(msg.header, msg.header.seq+1);

        return true;
      }

      /*!
       * \brief Update ROS color dwell message from Json entitlement value.
       *
       * \param [in]  jv  Json value of parsed entitlement.
       * \param [out] msg ROS data-specific dwell message.
       *
       * \return Returns true on success, false otherwise.
       */
      bool updateDwellColorMsg(const Json::Value &jv,
                               GfDwellColor    &msg)
      {
        updateEntitlementHeader(msg.entitlement.ent_header);

        msg.entitlement.color.r = (float)jv["color_red"].asUInt();
        msg.entitlement.color.g = (float)jv["color_green"].asUInt();
        msg.entitlement.color.b = (float)jv["color_blue"].asUInt();
        msg.entitlement.color.a = (float)jv["color_alpha"].asUInt();

        stampHeader(msg.header, msg.header.seq+1);

        return true;
      }

      /*!
       * \brief Update ROS profile dwell message from Json entitlement value.
       *
       * \param [in]  jv  Json value of parsed entitlement.
       * \param [out] msg ROS data-specific dwell message.
       *
       * \return Returns true on success, false otherwise.
       */
      bool updateDwellProfileMsg(const Json::Value &jv,
                                 GfDwellProfile    &msg)
      {
        updateEntitlementHeader(msg.entitlement.ent_header);

        msg.entitlement.gf_profile_num = jv["profile_num"].asUInt64();

        stampHeader(msg.header, msg.header.seq+1);

        return true;
      }

      /*!
       * \brief Update ROS threshold dwell message from Json entitlement value.
       *
       * \param [in]  jv  Json value of parsed entitlement.
       * \param [out] msg ROS data-specific dwell message.
       *
       * \return Returns true on success, false otherwise.
       */
      bool updateDwellThresholdMsg(const Json::Value &jv,
                                   GfDwellThreshold  &msg)
      {
        updateEntitlementHeader(msg.entitlement.ent_header);

        msg.entitlement.threshold_lower = jv["threshold_lower"].asDouble();
        msg.entitlement.threshold_upper = jv["threshold_upper"].asDouble();
        msg.entitlement.threshold_unit  = jv["threshold_unit"].asDouble();

        stampHeader(msg.header, msg.header.seq+1);

        return true;
      }

      /*!
       * \brief Update ROS Json dwell message from Json entitlement value.
       *
       * \param [in]  jv  Json value of parsed entitlement.
       * \param [out] msg ROS data-specific dwell message.
       *
       * \return Returns true on success, false otherwise.
       */
      bool updateDwellJsonMsg(const Json::Value &jv,
                              GfDwellJson       &msg)
      {
        Json::FastWriter  fastWriter;

        updateEntitlementHeader(msg.entitlement.ent_header);

        msg.entitlement.json = fastWriter.write(jv);

        stampHeader(msg.header, msg.header.seq+1);

        return true;
      }

      /*!
       * \brief Update entitlement metadata header.
       *
       * \param [in,out] header   Metadata header.
       */
      void updateEntitlementHeader(geofrenzy::GfEntHeader &header)
      {
        header.access_time  = m_atime;
        header.gf_class_idx = m_class_idx;
        header.gf_ent_idx   = m_ent_idx;
        header.gf_ent_base  = m_ent_base;
        header.dwell        = m_bDwell;
      }

      /*!
       * \brief Make dwell topic name.
       *
       * Note: Server name is automatically prepended.
       *
       * \param gf_ent_idx    Geofrenzy entitlement index.
       * \param gf_ent_base   Geofrenzy entitlement base.
       */
      std::string makeDwellTopicName(const uint64_t    gf_ent_idx,
                                     const std::string gf_ent_base)
      {
        std::stringstream  ss;
    
        ss << "geofrenzy/" << gf_ent_idx << "/dwell/" << gf_ent_base;
    
        return ss.str();
      }

  }; // class Entitlement


  //----------------------------------------------------------------------------
  // FenceServer Class
  //----------------------------------------------------------------------------

  /*!
   * \brief Class that implements the GeoJson Fence Server.
   */
  class FenceServer
  {
    public:
      /*!
       * Entitlement map type and its iteraters.
       */
      typedef std::map<uint64_t, geofrenzy::Entitlement*> EntitlementMap;
      typedef EntitlementMap::iterator        EntitlementMapIter;
      typedef EntitlementMap::const_iterator  EntitlementMapCIter;

      /*!
       * \brief Default initialization constructor.
       *
       * \param gf_class_idx  Associated geofrenzy class index for this node.
       * \param nh            ROS node handle.
       * \param hz            Node hertz rate.
       */
      FenceServer(ros::NodeHandle &nh, double hz, uint64_t gf_class_idx) :
          m_fenceClass(gf_class_idx), m_nh(nh), m_hz(hz), m_atime(0.0)
      {
        ROS_DEBUG_STREAM("FenceServer gf_class_idx = " << m_fenceClass);

        // current and previous positions are unknown
        m_previousLat  = NoGeoPos;
        m_previousLong = NoGeoPos;
        m_currentLat   = NoGeoPos;
        m_currentLong  = NoGeoPos;

        m_nPublishCnt = 0;
      };
  
      /*!
       * \brief Destructor.
       */
      ~FenceServer()
      {
        EntitlementMapIter  iter;

        for(iter = m_entitlements.begin(); iter != m_entitlements.end(); ++iter)
        {
          if( iter->second != NULL )
          {
            delete iter->second;
            iter->second = NULL;
          }
        }
      }

      /*!
       * \brief Intiialize the class properties for this node's associated
       * Geofrenzy class index.
       *
       * The initialization makes use of the Geofrenzy portal to retrieve the
       * relevant properties.
       *
       * \return Returns true on success, false otherwise.
       */
      bool initGfClassProperties()
      {
        Json::Reader  reader;   // json reader
        Json::Value   root;     // json parsed string root
        bool          bSuccess; // [not] successful

        ROS_INFO_STREAM(ros::this_node::getName()
            << ": Retrieving Geofrenzy properties for fence class "
            << m_fenceClass
            << ".");

        // retrieve the class properties
        char *td = class_entitlements_properties_json(m_fenceClass);

        if( td == NULL )
        {
          ROS_ERROR_STREAM(
              "Failed to retrieve class properties for class index "
              << m_fenceClass);
          return false;
        }

        ROS_INFO_STREAM(ros::this_node::getName()
            << ": Initializing Geofrenzy properties.");

        std::string strJson("{");
        strJson += td;
        strJson += "}";

        bSuccess = reader.parse(strJson, root);

        // report to the user the failure and their locations in the document.
        if( !bSuccess )
        {
          ROS_ERROR_STREAM("Failed to parse configuration\n"
              << reader.getFormattedErrorMessages());
          return false;
        };

        // property class index.
        // RDK. Should this always equal the node's class index?
        int gf_class_idx = root["class_metadata"]["class_idx"].asUInt64();

        // list of entitlements
        Json::Value entitlements = root["class_metadata"]["entitlements"];

        //
        // Add entitlements to fence server.
        //
        for(int i = 0; i < entitlements.size(); ++i)
        {
          uint64_t    gf_ent_idx  = entitlements[i]["ent_idx"].asUInt64();
          std::string gf_ent_base = entitlements[i]["ent_base"].asString();
          ROS_INFO_STREAM("New Entitlement: " << gf_ent_idx << " : " << gf_ent_base);
          // new
          if( m_entitlements.find(gf_ent_idx) == m_entitlements.end() )
          {
            m_entitlements[gf_ent_idx] =
                  new geofrenzy::Entitlement(m_fenceClass,
                                             gf_ent_idx,
                                             gf_ent_base);
          }
        }

        return true;
      }

      /*!
       * \brief Advertise all server services.
       */
      void advertiseServices()
      {
        std::string strSvc;

        strSvc = ServiceNameGetEntitlement;
        m_services[strSvc] = m_nh.advertiseService(strSvc,
                                                  &FenceServer::getEntitlement,
                                                  &(*this));

        strSvc = ServiceNameGetEntitlementList;
        m_services[strSvc] = m_nh.advertiseService(strSvc,
                                              &FenceServer::getEntitlementList,
                                              &(*this));

        EntitlementMapIter  iter;


        for(iter = m_entitlements.begin(); iter != m_entitlements.end(); ++iter)
        {
          iter->second->advertiseServices(m_nh);
        }
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
      void advertisePublishers(int nQueueDepth=2)
      {
        EntitlementMapIter  iter;

        m_publishers[TopicNameFcJson] =
          m_nh.advertise<geofrenzy::GfJsonFeatureCollection>(TopicNameFcJson,
                                                             1, true);

        m_publishers[TopicNameFcGeo] =
          m_nh.advertise<geofrenzy::GfGeoFeatureCollection>(TopicNameFcGeo,
                                                            1, true);

        m_publishers[TopicNameFcDist] =
          m_nh.advertise<geofrenzy::GfDistFeatureCollection>(TopicNameFcDist,
                                                                    1, true);
        for(iter = m_entitlements.begin(); iter != m_entitlements.end(); ++iter)
        {
          iter->second->advertisePublishers(m_nh);
        }
      }

      /*!
       * \brief Subscribe to all topics.
       *
       * \param nQueueDepth   Maximum queue depth.
       */
      void subscribeToTopics(int nQueueDepth=5)
      {
        // fix location
        m_subscriptions[TopicNameFix] =
                    m_nh.subscribe(TopicNameFix, 1,
                                   &FenceServer::cbNavSatFix,
                                   &(*this));
      }

      /*!
       * \brief Publish.
       *
       * Call in main loop.
       */
      void publish()
      {
        if( m_nPublishCnt > 0 )
        {
          m_publishers[TopicNameFcJson].publish(m_msgFcJson);
          m_publishers[TopicNameFcGeo].publish(m_msgFcGeo);
          m_publishers[TopicNameFcDist].publish(m_msgFcDist);

          --m_nPublishCnt;
        }

        EntitlementMapIter  iter;

        for(iter = m_entitlements.begin(); iter != m_entitlements.end(); ++iter)
        {
          iter->second->shouldPublish();
          iter->second->publish();
        }
      }

      /*!
       * \brief Retrieve Geofrenzy Portal fence and entitlements.
       *
       * \param [in] latitude   Geographic latitude.
       * \param [in] longitude  Geographic longitude.
       * \param [in] level      RoI. RDK Units are kilometers?
       * \param [out] strJson   Json formatted result string.
       *
       * \return Returns true on success, false otherwise.
       */
      bool retrieveGfPortalFences(double latitude, double longitude,
                                  int level, std::string &strJson)
      {
        std::string filename;
        char        *s;
  
        //
        // Read from file. Great for testing.
        //
        if( m_nh.getParam(ParamNameFenceFilename, filename) )
        {
          ROS_DEBUG_STREAM("Reading file " << filename);

          std::ifstream in(filename.c_str());
          std::string message;
  
          while(!in.eof())
          {
            message.push_back(in.get());
          }
          in.close();
          strJson = message;
          ROS_DEBUG_STREAM("Done reading file.");
        }

        //
        // Retrieve from Geofrenzy portal.
        //
        else
        {
          ROS_DEBUG_STREAM("Checking fence roi for class " << m_fenceClass);

          ROS_DEBUG_STREAM("Calling ambient_fences_geojson_roi()");

          s = ambient_fences_geojson_roi(longitude, latitude, level,
                                         m_fenceClass);
          strJson = s;
          ROS_DEBUG_STREAM("Done.");
        }
  
        if( !strJson.empty() )
        {
          m_atime = ros::Time::now();
          return true;
        }
        else
        {
          return false;
        }
      }

    protected:

      /*!
       * \brief Process json encoded string of Geofrenzy fences.
       *
       * The json string was retrieved from either the portal or from a file.
       *
       * Modifies:
       *  - entitlement data
       *
       * \param [in]  strJson   Json encoded string.
       * \param [out] msgFcGeo  Geocentric feature collection message.
       *
       * \return Returns true on success, false otherwise.
       */
      bool processJsonFences(const std::string      &strJson,
                             GfGeoFeatureCollection &msgFcGeo)
      {
        Json::Reader  reader;   // json reader
        Json::Value   root;     // root value after parsing
        bool          bSuccess; // [not] successful

        // parse json string
        bSuccess = reader.parse(strJson, root);

        // report to the user the failure and their locations in the document.
        if( !bSuccess )
        {
          ROS_ERROR_STREAM("Failed to parse fences.\n"
                                << reader.getFormattedErrorMessages());
          return false;
        }
  
        // clear ros message fetures
        msgFcGeo.features.clear();

        GfGeoFeature  geoFeature;   // ros geographic feature
        GeoPolygon    geoPolygon;   // ros geographic polygon

        // json feature list
        Json::Value featureList = root["features"];

        //
        // Loop through json feature list.
        //
        for(Json::Value::iterator feature = featureList.begin();
            feature != featureList.end();
            ++feature)
        {
          // feature data
          Json::Value &metadata = (*feature)["properties"]["class_metadata"];
          Json::Value &geometry = (*feature)["geometry"]["coordinates"];
          Json::Value inout     = (*feature)["properties"]["inout"];

          Json::Value &entitlements  = metadata["entitlements"];

          bool bDwell = inout.asString().compare("i") == 0;

          // clear ros feature entitlements and geometries
          geoFeature.gf_ent_idx.clear();
          geoFeature.geometry.clear();

          geoFeature.gf_class_idx   = metadata["class_idx"].asUInt64();

          //
          // Loop through json feature fences.
          //
          for(Json::Value::iterator polygon_iter = geometry.begin();
              polygon_iter != geometry.end();
              ++polygon_iter)
          {
            Json::Value &polygon = (*polygon_iter);

            // clear polygon
            geoPolygon.points.clear();

            //
            // Loop through points outlining a polygonal fence.
            //
            for(Json::Value::iterator point_iter = polygon.begin();
                point_iter != polygon.end();
                ++point_iter)
            {
              geographic_msgs::GeoPoint geoPoint;

              geoPoint.longitude = (*point_iter)[0].asDouble();
              geoPoint.latitude  = (*point_iter)[1].asDouble();
              geoPoint.altitude  = 0.0;

              // add point
              geoPolygon.points.push_back(geoPoint);
            }

            // add polygon fence
            geoFeature.geometry.push_back(geoPolygon);
          }

          //
          // Loop the json feature entitlements.
          //
          for(Json::Value::iterator entitlement = entitlements.begin();
              entitlement != entitlements.end();
              ++entitlement)
          {
            uint64_t gf_ent_idx  = (*entitlement)["ent_idx"].asUInt64();

            if( m_entitlements.find(gf_ent_idx) != m_entitlements.end() )
            {
              m_entitlements[gf_ent_idx]->update(*entitlement, bDwell, m_atime);
            }

            // add entitlement index
            geoFeature.gf_ent_idx.push_back(gf_ent_idx);
          }
  
          // add feature
          msgFcGeo.features.push_back(geoFeature);
        }

        return true;
      }

      /*!
       * \brief Convert geocentric feature collection message data to fixed
       * location centric distance feature collection message.
       *
       * \param [in]  msgFcGeo  Populated geocentric feature collection message.
       * \param [out] msgFcDist Location centric distance feature collection
       *                        message.
       */
      void convertGeoToDistMsg(GfGeoFeatureCollection  &msgFcGeo,
                               GfDistFeatureCollection &msgFcDist)

      {
        msgFcDist.features.clear();

        GfDistFeature distFeature;
        Polygon64     distPolygon;

        //
        // Loop through the collection of features.
        //
        for(size_t i = 0; i < msgFcGeo.features.size(); ++i)
        {
          GfGeoFeature &geoFeature = msgFcGeo.features[i];

          distFeature.gf_class_idx = geoFeature.gf_class_idx;
          distFeature.gf_ent_idx.clear();
          distFeature.geometry.clear();

          //
          // Loop through the entitlements per feature.
          //
          for(size_t j = 0; j < geoFeature.gf_ent_idx.size(); ++j)
          {
            // add entitlement index
            distFeature.gf_ent_idx.push_back(geoFeature.gf_ent_idx[j]);
          }

          //
          // Loop through the fences per feature.
          //
          for(size_t j = 0; j < geoFeature.geometry.size(); ++j)
          {
            GeoPolygon &geoPolygon = geoFeature.geometry[j];

            distPolygon.points.clear();

            //
            // Loop through the points outlining a polygonal fence.
            //
            for(size_t k = 0; k < geoPolygon.points.size(); ++k)
            {
              geographic_msgs::GeoPoint &geoPoint = geoPolygon.points[k];
              geometry_msgs::Point      distPoint;

              // convert to distance and transform points to XY:NW coords
              swri_transform_util::LocalXyFromWgs84(
                        m_currentLat, m_currentLong,
                        geoPoint.latitude, geoPoint.longitude,
                        distPoint.y, distPoint.x);
              distPoint.y = distPoint.y;
              distPoint.x = -distPoint.x;
              distPoint.z  = geoPoint.altitude;

              // add point
              distPolygon.points.push_back(distPoint);
            }

            // add polygon fence
            distFeature.geometry.push_back(distPolygon);
          }

          // add feature
          msgFcDist.features.push_back(distFeature);
        }
      }

      /*!
       * \brief Navigation satellite fixed location subscribed callback.
       */
      void cbNavSatFix(const sensor_msgs::NavSatFix::ConstPtr &msg)
      {
        ROS_DEBUG_STREAM(TopicNameFix);

        std::string   strJson;
        double        deltaDist;
        double        paramMinDist;
        double        paramRoILevel;

        // no gps acquired
        if( msg->status.status == -1 )
        {
          ROS_INFO_STREAM("No GPS acquired.");
          return;
        }

        //Set time fix was acquired
        m_fixTime = msg->header.stamp;
  
        // new location
        m_previousLat  = m_currentLat;
        m_previousLong = m_currentLong;
        m_currentLat   = msg->latitude;
        m_currentLong  = msg->longitude;

        // no previous - set to current
        if( (m_previousLat == NoGeoPos) || (m_previousLong == NoGeoPos) )
        {
          m_previousLat  = m_currentLat;
          m_previousLong = m_currentLong;
        }

        ROS_INFO_STREAM("Previous lat,long: "
            << m_previousLat
            << ", "
            << m_previousLong);

        ROS_INFO_STREAM("     New lat,long: "
            << m_currentLat
            << ", "
            << m_currentLong);
  
  
        //
        // Calculate the delta distance from previous location.
        //
        deltaDist = swri_transform_util::GreatCircleDistance(
                              m_currentLat, m_currentLong,
                              m_previousLat, m_previousLong);

        ROS_DEBUG_STREAM("Delta distance = " << deltaDist);

        //
        // Get the minimum distance (meters) from Parameter Server.
        //
        // Note: This parameter can change underneath this node's execution.
        //
        m_nh.param(ParamNameMinDist, paramMinDist, MinDistDft);
  
        //
        // Distance threshold - no publishing new data if within.
        //
        if( deltaDist < paramMinDist )
        {
          ROS_DEBUG_STREAM("Delta distance "
              << deltaDist
              << " < minimum "
              << paramMinDist);
          return;
        }
  
        //
        // Get the region of interest (meters) from Parameter Server.
        //
        // Note: This parameter can change underneath this node's execution.
        //
        m_nh.param(ParamNameRoILevel, paramRoILevel, RoILevelDft);
  
        //
        // Retrieve all fences (geometry and entitlements) within the RoI
        // from the Geofrenzy portal.
        //
        // Note: Geofrenzy RoI is in kilometers.
        //
        if( !retrieveGfPortalFences(m_currentLat, m_currentLong,
                                    (int)(paramRoILevel/1000.0), strJson) )
        {
          ROS_ERROR("Failed to retrieve Geofrenzy fences.");
          return;
        }

        //
        // Process json fences, including updating entitlemnt data.
        //
        if( !processJsonFences(strJson, m_msgFcGeo) )
        {
          ROS_ERROR("Failed to process Geofrenzy fences.");
          return;
        }

        //
        // Update all feature collection published messages.
        //
        updateFeatureCollectionMsgs(strJson);
      };

      /*!
       * \brief Update feature collection messages from current location.
       */
      void updateFeatureCollectionMsgs(const std::string &strJson)
      {
        // Json feature collection
        m_msgFcJson.features.data = strJson;
        m_msgFcJson.access_time   = m_atime;
        m_msgFcJson.fix_time = m_fixTime;
        stampHeader(m_msgFcJson.header, m_msgFcJson.header.seq+1);

        // geographic centric feature collection (data filled on update)
        m_msgFcGeo.access_time = m_atime;
        m_msgFcGeo.fix_time = m_fixTime;
        stampHeader(m_msgFcGeo.header, m_msgFcGeo.header.seq+1);

        // distance centric feature collection
        convertGeoToDistMsg(m_msgFcGeo, m_msgFcDist);
        m_msgFcDist.access_time = m_atime;
        m_msgFcDist.fix_time = m_fixTime;
        stampHeader(m_msgFcDist.header, m_msgFcDist.header.seq+1);

        ++m_nPublishCnt;
      }

      /*!
       * \brief Get the Goefrenzy entitlement callback.
       *
       * \param req   Service request.
       * \param rsp   Service response.
       *
       * \return Returns true on success, false on failure.
       */
      bool getEntitlement(geofrenzy::GetEntitlement::Request  &req,
                          geofrenzy::GetEntitlement::Response &rsp)
      {
        ROS_DEBUG_STREAM(ServiceNameGetEntitlement);

        if( m_entitlements.find(req.gf_ent_idx) == m_entitlements.end() )
        {
          ROS_ERROR_STREAM("Service " << ServiceNameGetEntitlement << ": "
              << "Requesting entitlement "
              << req.gf_ent_idx
              << " not found.");
          return false;
        }

        m_entitlements[req.gf_ent_idx]->fill(rsp);

        return true;
      };

      /*!
       * \brief Get the Goefrenzy entitlement list callback.
       *
       * \param req   Service request.
       * \param rsp   Service response.
       *
       * \return Returns true on success, false on failure.
       */
      bool getEntitlementList(geofrenzy::GetEntitlementList::Request  &req,
                              geofrenzy::GetEntitlementList::Response &rsp)
      {
        ROS_DEBUG_STREAM(ServiceNameGetEntitlementList);
      
        EntitlementMapCIter iter;

        rsp.gf_ent_idx.clear();
        rsp.gf_ent_base.clear();

        for(iter = m_entitlements.begin(); iter != m_entitlements.end(); ++iter)
        {
          rsp.gf_ent_idx.push_back(iter->second->entitlementIndex());
          rsp.gf_ent_base.push_back(iter->second->entitlementBase());
        }

        return true;
      }
  
    private:
      ros::NodeHandle &m_nh;            ///< node handle
      double          m_hz;             ///< cycle hertz
      uint64_t        m_fenceClass;    ///< geofrenzy class index

      // ROS services, publishers, subscriptions.
      MapServices       m_services;       ///< Geofrenzy server services
      MapClientServices m_clientServices; ///< Geofrenzy server client services
      MapPublishers     m_publishers;     ///< Geofrenzy server publishers
      MapSubscriptions  m_subscriptions;  ///< Geofrenzy server subscriptions

      // Satellite navigation fix
      double    m_previousLat;     ///< previous latitude
      double    m_previousLong;    ///< previous longitude
      double    m_currentLat;      ///< current latitude
      double    m_currentLong;     ///< current longitude

      // Messaging processing overhead
      ros::Time m_atime;        ///< last portal access time
      ros::Time m_fixTime;      ///< last NavSat fix time
      int       m_nPublishCnt;  ///< publish counter

      // Message for publishing
      GfGeoFeatureCollection  m_msgFcGeo;   ///< geographical feature collection
      GfDistFeatureCollection m_msgFcDist;  ///< distance feature collection
      GfJsonFeatureCollection m_msgFcJson;  ///< Json encode feature collection

      // Geofrenzy entitlements
      EntitlementMap  m_entitlements; ///< class entitlements
  
  }; // class FenceServer

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
  double  hz;
  double  paramMinDist;

  // get command-line Geofrenzy class index
  uint64_t gfClassIdx = paramClassIndex(argc, argv);

  // make a unique node name from the command line class index argument
  std::string nodeName = makeNodeName(NodeRootFenceServer, gfClassIdx);

  // 
  // Initialize the node. Parse the command line arguments and environment to
  // determine ROS options such as node name, namespace and remappings.
  // This call does not contact the master. This lets you use
  // ros::master::check() and other ROS functions after calling ros::init()
  // to check on the status of the master.
  //
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
    return 0;
  }

  ROS_INFO_STREAM(nodeName << ": ROS master running.");

  //
  // Create fence server node.
  //
  geofrenzy::FenceServer fs(nh, hz, gfClassIdx);

  //
  // Initialize fence server.
  //
  if( !fs.initGfClassProperties() )
  {
    ROS_FATAL_STREAM(nodeName << ": Failed to initialize.");
    return 2;
  }

  ROS_INFO_STREAM(nodeName << ": Node initialized.");

  //
  // Advertise services.
  //
  fs.advertiseServices();

  ROS_INFO_STREAM(nodeName << ": Advertised services registered.");

  //
  // Advertise publishers.
  //
  fs.advertisePublishers();
  
  ROS_INFO_STREAM(nodeName << ": Advertised publishers registered.");
  
  //
  // Subscribed to topics.
  //
  fs.subscribeToTopics();
  
  ROS_INFO_STREAM(nodeName << ": Subscribed topics registered.");

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
    fs.publish();

    // sleep to keep at loop rate
    loop_rate.sleep();
  }

  return 0;
}
