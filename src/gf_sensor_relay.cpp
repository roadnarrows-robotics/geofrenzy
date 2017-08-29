////////////////////////////////////////////////////////////////////////////////
//
// Package:   Geofrenzy Robot ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/geofrenzy
//
// File:      gf_sensor_relay.cpp
//
/*! \file
 *
 * \brief The Geofrenzy ROS sensor relay node source.
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
#include <vector>

//
// ROS
//
#include "ros/ros.h"
#include "ros/console.h"

//
// ROS generated core messages
//

//
// mavros
//

//
// ROS generated Geofrenzy messages
//
#include "geofrenzy/GfDwellBoolset.h"
#include "geofrenzy/GfDwellProfile.h"
#include "geofrenzy/GfDwellThreshold.h"
#include "geofrenzy/GfDwellJson.h"

//
// Geofrenzy
//
#include "gf_types.h"
#include "gf_ros.h"
#include "gf_sentinel.h"

using namespace geofrenzy::gf_ros;

namespace geofrenzy
{
  typedef std::vector<GfSentinel*>      SentinelList;
  typedef SentinelList::iterator        SentinelIter;
  typedef SentinelList::const_iterator  SentinelCIter;

  class SensorRelay
  {
    /*!
    * This class is responsible for registering incoming/outgoing
    * ros topics that are controlled by geofrenzy entitlements.
    */
    public:
        void dwellCallbackBoolset(const GfDwellBoolset  &dwellMsg);
        void dwellCallbackProfile(const GfDwellProfile  &dwellMsg);
        void dwellCallbackThreshold(const GfDwellThreshold &dwellMsg);
        void advertisePublishers(int nQueueDepth=1);
        void subscribeToTopics(int nQueueDepth=5);
        void initSensorRelayProperties();

        SensorRelay(ros::NodeHandle &nh) : m_nh(nh){}

        ~SensorRelay();

    private:
        ros::NodeHandle &m_nh;    ///< node handle
        GfClassIndex m_gciServer; ///< geofrenzy portal server node class index

        //
        // Embedded sentinels
        //
        SentinelList  m_sentinels;

        //ROS publishers/subscribers
        gf_ros::MapPublishers     m_publishers;     ///< topic publishers
        gf_ros::MapSubscriptions  m_subscriptions;  ///< topic subscriptions
  };

  SensorRelay::~SensorRelay()
  {
    for(size_t i = 0; i < m_sentinels.size(); ++i)
    {
      delete m_sentinels[i];
      m_sentinels[i] = NULL;
    }
    m_sentinels.clear();
  }

  /*!
   * \brief Init properties. Check for topic names on param server.
   *
   */
  void SensorRelay::initSensorRelayProperties()
  {
    GfSentinel  *p;

    //
    // Get the gefrenzy portal server class index.
    // 
    // Note: no 64-bit types in parameter server
    //
    int32_t   val;
    int32_t   dft = (int32_t)GciGuestAcct;

    m_nh.param(ParamNameSrServerGci, val, dft);

    m_gciServer = (GfClassIndex)val;

    //
    // Built-in sentinels
    //
    m_sentinels.push_back(new GfSentinelCam());
    m_sentinels.push_back(new GfSentinelStop());
    //m_sentinels.push_back(new GfSentinelSpeedLimitVel());
    m_sentinels.push_back(new GfSentinelMavRtl());

    //
    // Initialize properties of all built-in sentinels
    //
    for(size_t i = 0; i < m_sentinels.size(); ++i)
    {
      m_sentinels[i]->initProperties(m_nh, m_gciServer);
      std::cerr << *m_sentinels[i];
    }
  }


  /*!
   * \brief Subscribe to all topics.
   *
   * \param nQueueDepth   Maximum queue depth.
   */
  void SensorRelay::subscribeToTopics(int nQueueDepth)
  {
    size_t  i, j;

    //
    // Sentinel specific topic subscriptions
    //
    for(i = 0; i < m_sentinels.size(); ++i)
    {
      m_sentinels[i]->subscribeToTopics(m_nh);
    }

    //
    // Geofrenzy dwell topic subscriptions. Driven by sentinel needs.
    //
    for(i = 0; i < m_sentinels.size(); ++i)
    {
      for(j = 0; j < m_sentinels[i]->eoiSize(); ++j)
      {
        const GfSentinel::EoI &eoi = m_sentinels[i]->eoiAt(j);

        // already subscribed
        if( m_subscriptions.find(eoi.m_topicDwell) != m_subscriptions.end() )
        {
          continue;
        }

        // subscribe
        switch( eoi.m_entDataType )
        {
          case GfEntDataTypeBoolset:
            m_subscriptions[eoi.m_topicDwell] = m_nh.subscribe(
                                          eoi.m_topicDwell,
                                          nQueueDepth,
                                          &SensorRelay::dwellCallbackBoolset,
                                          &(*this));
            break;
          case GfEntDataTypeJson:
            ROS_WARN("Json dwell topic not supported yet.");
            break;
          case GfEntDataTypeProfile:
            m_subscriptions[eoi.m_topicDwell] = m_nh.subscribe(
                                          eoi.m_topicDwell,
                                          nQueueDepth,
                                          &SensorRelay::dwellCallbackProfile,
                                          &(*this));
            break;
          case GfEntDataTypeThreshold:
            m_subscriptions[eoi.m_topicDwell] = m_nh.subscribe(
                                          eoi.m_topicDwell,
                                          nQueueDepth,
                                          &SensorRelay::dwellCallbackThreshold,
                                          &(*this));
            break;
          case GfEntDataTypeUndef:
            ROS_WARN("Undefined entitlement data type. "
                     "Not subscribing to topic.");
            break;
          default:
            ROS_ERROR_STREAM("Unknnown entitlement data type = "
                << eoi.m_entDataType << ".");
            break;
        }
      }
    }
  }

  /*!
   * \brief Advertise all publishers.
   *
   * \param nQueueDepth   Maximum queue depth.
   */
  void SensorRelay::advertisePublishers(int nQueueDepth)
  {
    //
    // Sentinel specific published topics
    //
    for(size_t i = 0; i < m_sentinels.size(); ++i)
    {
      m_sentinels[i]->advertisePublishers(m_nh);
    }
  }

  void SensorRelay::dwellCallbackBoolset(const GfDwellBoolset &dwellMsg)
  {
    const GfEntHeader &hdr = dwellMsg.entitlement.ent_header;

    GfEntDataType     t = GfEntDataTypeBoolset;
    GfEntBaseBoolset  d = dwellMsg.entitlement.bitset_num;

    SentinelIter  iter;

    for(iter = m_sentinels.begin(); iter != m_sentinels.end(); ++iter)
    {
      if( (*iter)->eoiCheck(hdr.gf_class_idx, hdr.gf_ent_idx, t) )
      {
        (*iter)->cbWatchForBreach(hdr.gf_ent_idx, hdr.dwell, d);
      }
    }
  }

  void SensorRelay::dwellCallbackProfile(const GfDwellProfile &dwellMsg)
  {
    const GfEntHeader &hdr = dwellMsg.entitlement.ent_header;


    GfEntDataType     t = GfEntDataTypeProfile;
    GfEntBaseProfile  d = dwellMsg.entitlement.gf_profile_num;

    SentinelIter  iter;

    for(iter = m_sentinels.begin(); iter != m_sentinels.end(); ++iter)
    {
      if( (*iter)->eoiCheck(hdr.gf_class_idx, hdr.gf_ent_idx, t) )
      {
        (*iter)->cbWatchForBreach(hdr.gf_ent_idx, hdr.dwell, d);
      }
    }
  }

  void SensorRelay::dwellCallbackThreshold(const GfDwellThreshold &dwellMsg)
  {
    const GfEntHeader &hdr = dwellMsg.entitlement.ent_header;

    GfEntDataType       t = GfEntDataTypeThreshold;
    GfEntBaseThreshold  d;

    d.m_upper = dwellMsg.entitlement.threshold_lower;
    d.m_lower = dwellMsg.entitlement.threshold_upper;
    d.m_units = dwellMsg.entitlement.threshold_unit;

    SentinelIter  iter;

    for(iter = m_sentinels.begin(); iter != m_sentinels.end(); ++iter)
    {
      if( (*iter)->eoiCheck(hdr.gf_class_idx, hdr.gf_ent_idx, t) )
      {
        (*iter)->cbWatchForBreach(hdr.gf_ent_idx, hdr.dwell, d);
      }
    }
  }

} // namespace geofrenzy


/**
 * This node subscribes to dwell topics and determines permissions for
 * various robot sensors. The node subscribes to relevant sensor
 * topics and either relays them or not based on permissions. This is
 * not likely to be the permanent solution, as gf_server will resolve
 * permissions and publish topics targeted at specific subsystems.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, NodeRootSensorRelay);

    ros::NodeHandle nh(NodeRootSensorRelay);

    geofrenzy::SensorRelay sr(nh);
    sr.initSensorRelayProperties();
    sr.advertisePublishers();
    sr.subscribeToTopics();

    while(ros::ok()){
        ros::spin();
    }
}
