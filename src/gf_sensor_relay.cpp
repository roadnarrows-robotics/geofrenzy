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
        void dwellCallbackBoolset(const GfDwellBoolset &dwell);
        void advertisePublishers(int nQueueDepth=1);
        void subscribeToTopics(int nQueueDepth=5);
        void initSensorRelayProperties();

        SensorRelay(ros::NodeHandle &nh) : m_nh(nh){}

        ~SensorRelay();

    private:

        ros::NodeHandle &m_nh;      ///< node handle

        GfClassIndex m_gciServer;   ///< geofrenzy server node class index

        //
        // Built-in breaches
        //
        SentinelList  m_breaches;

        //ROS publishers/subscribers
        gf_ros::MapPublishers     m_publishers;     ///< topic publishers
        gf_ros::MapSubscriptions  m_subscriptions;  ///< topic subscriptions

  };

  SensorRelay::~SensorRelay()
  {
    for(size_t i = 0; i < m_breaches.size(); ++i)
    {
      delete m_breaches[i];
      m_breaches[i] = NULL;
    }
    m_breaches.clear();
  }

  /*!
   * \brief Init properties. Check for topic names on param server.
   *
   */
  void SensorRelay::initSensorRelayProperties()
  {
    int32_t   val;
    int32_t   dft = (int32_t)GciGuestAcct;
    GfSentinel  *p;

    //
    // Get the gefrenzy portal server class index.
    // 
    // Note: no 64-bit types in parameter server
    //
    m_nh.param(ParamNameSrServerGci, val, dft);

    m_gciServer = (GfClassIndex)val;

    //
    // Built-in breaches
    //
    m_breaches.push_back(new GfSentinelCam());

    //
    // Initialize properties of all built-in breaches
    for(size_t i = 0; i < m_breaches.size(); ++i)
    {
      m_breaches[i]->initProperties(m_nh, m_gciServer);
    }
  }


  /*!
   * \brief Subscribe to all topics.
   *
   * \param nQueueDepth   Maximum queue depth.
   */
  void SensorRelay::subscribeToTopics(int nQueueDepth)
  {
    size_t  i;

    //
    // Sentinel specific topic subscriptions
    //
    for(i = 0; i < m_breaches.size(); ++i)
    {
      m_breaches[i]->subscribeToTopics(m_nh);
    }

    //
    // Geofrenzy dwell topic subscriptions
    //
    for(i = 0; i < m_breaches.size(); ++i)
    {
      std::string &topic = m_breaches[i]->m_topicDwell;

      // nothing to subscribe
      if( topic.empty() )
      {
        continue;
      }

      // already subscribed
      else if( m_subscriptions.find(topic) != m_subscriptions.end() )
      {
        continue;
      }

      // subscribe
      else
      {
        switch( m_breaches[i]->m_entDataType )
        {
          case GfEntDataTypeBoolset:
            m_subscriptions[topic] = m_nh.subscribe(topic,
                                            nQueueDepth,
                                            &SensorRelay::dwellCallbackBoolset,
                                            &(*this));
            break;
          case GfEntDataTypeUndef:
            ROS_INFO("Undefined entitiy data type. Not subscribing to topic");
            break;
          default:
            ROS_ERROR_STREAM("Unknnown entitiy data type = "
                << m_breaches[i]->m_entDataType << ".");
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
    for(size_t i = 0; i < m_breaches.size(); ++i)
    {
      m_breaches[i]->advertisePublishers(m_nh);
    }
  }

  void SensorRelay::dwellCallbackBoolset(const GfDwellBoolset &dwellMsg)
  {
    for(size_t i = 0; i < m_breaches.size(); ++i)
    {
      GfSentinel *b = m_breaches[i];

      if( (dwellMsg.entitlement.ent_header.gf_class_idx == b->m_gci) &&
          (dwellMsg.entitlement.ent_header.gf_ent_idx   == b->m_gei) &&
          (b->m_entDataType == GfEntDataTypeBoolset) )
      {
        b->cbWatchForBreach(dwellMsg.entitlement.ent_header.dwell);
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

    ros::NodeHandle nh;

    geofrenzy::SensorRelay sr(nh);
    sr.initSensorRelayProperties();
    sr.advertisePublishers();
    sr.subscribeToTopics();

    while(ros::ok()){
        ros::spin();
    }
}
