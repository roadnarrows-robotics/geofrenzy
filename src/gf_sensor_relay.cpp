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
#include <ostream>

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
        void advertiseServices();
        void clientServices();

        void initSensorRelayProperties();

        SensorRelay(ros::NodeHandle &nh) : m_nh(nh){}

        ~SensorRelay();

        friend std::ostream &operator<<(std::ostream &os,
                                        const SensorRelay &sr);

    private:
        ros::NodeHandle &m_nh;      ///< node handle

        GfClassIndex  m_gciServer;  ///< geofrenzy portal server class index
        int32_t       m_enables;    ///< built-in enables

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
    int32_t     val;
    int32_t     dft = (int32_t)GciGuestAcct;


    //
    // Get the gefrenzy portal server class index.
    // 
    // Note: no 64-bit types in parameter server
    //
    dft = (int32_t)GciGuestAcct;
    m_nh.param(ParamNameSrServerGci, val, dft);

    m_gciServer = (GfClassIndex)val;

    //
    // Get the gefrenzy built-in sentinel enable/disable states.
    // 
    //
    dft = (int32_t)SrSentinelEnableAll;
    m_nh.param(ParamNameSrEnables, m_enables, dft);

    //
    // Built-in sentinels
    //
    if( m_enables & SrSentinelEnableCam )
    {
      m_sentinels.push_back(new GfSentinelCam());
    }

    if( m_enables & SrSentinelEnableStop )
    {
      m_sentinels.push_back(new GfSentinelStop());
    }

    if( m_enables & SrSentinelEnableSpeed )
    {
      //m_sentinels.push_back(new GfSentinelSpeedLimitVel());
    }

    if( m_enables & SrSentinelEnableMav )
    {
      m_sentinels.push_back(new GfSentinelMav());
    }

    //
    // Initialize properties of all built-in sentinels
    //
    for(size_t i = 0; i < m_sentinels.size(); ++i)
    {
      m_sentinels[i]->initProperties(m_nh, m_gciServer);
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

  /*!
   * \brief Advertise all services.
   */
  void SensorRelay::advertiseServices()
  {
    //
    // Sentinel specific services
    //
    for(size_t i = 0; i < m_sentinels.size(); ++i)
    {
      m_sentinels[i]->advertiseServices(m_nh);
    }
  }

  /*!
   * \brief Intialize all client services.
   */
  void SensorRelay::clientServices()
  {
    //
    // Sentinel specific client services
    //
    for(size_t i = 0; i < m_sentinels.size(); ++i)
    {
      m_sentinels[i]->clientServices(m_nh);
    }
  }

  void SensorRelay::dwellCallbackBoolset(const GfDwellBoolset &dwellMsg)
  {
    //ROS_INFO_STREAM("dwellCallbackBoolset = " << dwellMsg);

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
    //ROS_INFO_STREAM("dwellCallbackThreshold = " << dwellMsg);

    const GfEntHeader &hdr = dwellMsg.entitlement.ent_header;

    GfEntDataType       t = GfEntDataTypeThreshold;
    GfEntBaseThreshold  d;

    d.m_lower = dwellMsg.entitlement.threshold_lower;
    d.m_upper = dwellMsg.entitlement.threshold_upper;
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

  std::ostream &operator<<(std::ostream &os, const SensorRelay &sr)
  {
    os << "SensorRelay:" << std::endl
      << "  gciServer = " << sr.m_gciServer << std::endl
      << "  sentinels[" << sr.m_sentinels.size() << "] =" << std::endl;

    for(size_t i = 0; i < sr.m_sentinels.size(); ++i)
    {
      os << *(sr.m_sentinels[i]) << std::endl;
    }

    os  << "dwellSubscriptions:" << std::endl
        << "  topics[" << sr.m_subscriptions.size() << "] =" << std::endl
        << "  {" << std::endl;
    for(MapSubscriptions::const_iterator iter = sr.m_subscriptions.begin();
        iter != sr.m_subscriptions.end();
        ++iter)
    {
      os << "    " << iter->first << std::endl;
    }
    os  << "  }" << std::endl;

    return os;
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
    sr.advertiseServices();
    sr.clientServices();

    std::cerr << sr << std::endl;

    while(ros::ok()){
        ros::spin();
    }
}
