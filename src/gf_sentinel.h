////////////////////////////////////////////////////////////////////////////////
//
// Package:   Geofrenzy Robot ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/geofrenzy
//
// File:      gf_sentinel.h
//
/*! \file
 *
 * \brief The Geofrenzy sentinel base and derived built-in class
 * interfaces.
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

#ifndef _GF_SENTINEL_H
#define _GF_SENTINEL_H

//
// System
//
#include <string>

//
// ROS
//
#include "ros/ros.h"
#include "ros/console.h"

//
// ROS generated core messages
//
#include "sensor_msgs/Image.h"

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

namespace geofrenzy
{
  // ---------------------------------------------------------------------------
  // GfSentinel Base Class
  // ---------------------------------------------------------------------------

  /*!
   * \brief Geofrenzy sentinel base class.
   *
   * A sentinal watches for and act on breaches of entitlements associated with
   * a set of geofences
   *
   * A breach may occur when a mobile system enters and/or exits a fence.
   * The action taken on a breach event is specific to the derived sentinel.
   * Further action may be taken if a breach is remediated.
   */
  class GfSentinel
  {
  public:
    /*!
     * \brief Sentinel ROS-centric Message Exchange Patterns.
     */
    enum Mep
    {
      MepUndef,     ///< undefined
      MepSubPub,    ///< subscribed and published topics (bent pipe)
      MepSubSvc,    ///< subscribed topic and service request action
      MepSvcSvc,    ///< service request relay (proxy)
      MepSvc,       ///< service request action
      MepCall       ///< call underlining api (may not really be a message)
    };

    /*!
     * \brief Sentinel breach action categories.
     */
    enum BreachAction
    {
      BreachActionUndef,  ///< undefined
      BreachActionCensor, ///< censor mobile system component(s)
      BreachActionRtl,    ///< mobile system return to landing
      BreachActionStop    ///< mobile system all stop
    };

    //
    // Sentinel meta-data
    //
    GfClassIndex        m_gci;
    GfEntitlementIndex  m_gei;
    GfEntDataType       m_entDataType;  ///< base data type for dwell message
    Mep                 m_mep;
    BreachAction        m_breachAction;

    //
    // Sentinel state
    //
    bool  m_isInFence;    ///< mobile device is [not] in a geofence
    bool  m_isInBreach;   ///< mobile device is [not] in breach of entitlements

    //
    // ROS topic and service names
    //
    std::string   m_topicDwell;   ///< dwell message topic
    std::string   m_topicIn;      ///< subscribed topic
    std::string   m_topicOut;     ///< published topic
    std::string   m_serviceIn;    ///< service in
    std::string   m_serviceOut;   ///< service out

    GfSentinel();

    virtual ~GfSentinel();

    virtual void initProperties(ros::NodeHandle &nh, GfClassIndex gci);

    virtual void subscribeToTopics(ros::NodeHandle &nh, int nQueueDepth=1);

    virtual void advertisePublishers(ros::NodeHandle &nh, int nQueueDepth=1);

    virtual void cbWatchForBreach(bool isInFence);

    void clear();
    
  protected:
    gf_ros::MapPublishers     m_publishers;     ///< topic publishers
    gf_ros::MapSubscriptions  m_subscriptions;  ///< topic subscriptions
  };


  // ---------------------------------------------------------------------------
  // GfSentinelCam Derived Class
  // ---------------------------------------------------------------------------

  class GfSentinelCam : public GfSentinel
  {
  public:
    GfSentinelCam();

    virtual ~GfSentinelCam();

    virtual void initProperties(ros::NodeHandle &nh, GfClassIndex gci);

    virtual void subscribeToTopics(ros::NodeHandle &nh, int nQueueDepth=5);

    virtual void advertisePublishers(ros::NodeHandle &nh, int nQueueDepth=1);

    virtual void cbWatchForBreach(bool isInFence);

  protected:
    sensor_msgs::Image m_imgCensored; ///< image to publish when camera is not
                                      ///< permitted

    virtual void makeCensoredImage(ros::NodeHandle &nh);

    virtual void cbImage(const sensor_msgs::Image &img);
  };

} // namespace geofrenzy

#endif // _GF_SENTINEL_H
