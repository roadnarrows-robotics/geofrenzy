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
#include <stddef.h>

#include <string>
#include <ostream>

//
// ROS
//
#include "ros/ros.h"
#include "ros/console.h"

//
// ROS generated core messages
//
#include "sensor_msgs/Image.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"

//
// mavros
//
#include "mavros_msgs/State.h"
#include "mavros_msgs/ExtendedState.h"
#include "mavros_msgs/HomePosition.h"

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

namespace geofrenzy
{
  // ---------------------------------------------------------------------------
  // GfSentinel Base Class
  // ---------------------------------------------------------------------------

  /*!
   * \brief Geofrenzy sentinel base class.
   *
   * A sentinal watches for breaches of entitlements associated with a set of
   * geofences. When a breach occurs (or is remediated), a sentinel takes the
   * appropriate action.
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
     * \brief Sentinel breach triggering event.
     */
    enum BreachTrigger
    {
      BreachTriggerUndef, ///< undefined
      BreachTriggerEntry, ///< breach is triggered on geofence entry
      BreachTriggerExit   ///< breach is triggered on geofence exit
    };

    /*!
     * \brief Sentinel breach action categories.
     */
    enum BreachAction
    {
      BreachActionUndef,      ///< undefined
      BreachActionCensor,     ///< censor mobile system component(s)
      BreachActionRtl,        ///< mobile system return to landing
      BreachActionStop,       ///< mobile system all stop
      BreachActionLimitSpeed  ///< mobile system all stop
    };

    /*!
     * \brief Entitlement of Interest structure.
     */
    struct EoI
    {
      GfEntitlementIndex  m_gei;          ///< entitlement index 
      GfEntDataType       m_entDataType;  ///< base data type for dwell message
      std::string         m_topicDwell;   ///< dwell message topic
    };

    typedef std::vector<EoI>        EoIList;  ///< EoI list data type
    typedef EoIList::iterator       EoIIter;  ///< EoI list iterator type
    typedef EoIList::const_iterator EoICIter; ///< EoI list const iterator type

    /*!
     * \brief Default constructor.
     */
    GfSentinel();

    /*!
     * \brief Destructor.
     */
    virtual ~GfSentinel();

    /*!
     * \brief Initialize all quasi-static properties.
     *
     * \param nh    Relay ROS node handle of the embedded sentinel.
     * \param gci   Geofrenzy class index of ROS portal server.
     */
    virtual void initProperties(ros::NodeHandle &nh, GfClassIndex gci);

    /*!
     * \brief Subscribe to all topics.
     *
     * Excluded are Geofrenzy dwell breach topics which are handle by the
     * sensor relay node.
     *
     * \param nh          Relay ROS node handle of the embedded sentinel.
     * \param nQueueDepth Maximum receive queue depth.
     */
    virtual void subscribeToTopics(ros::NodeHandle &nh, int nQueueDepth=1)
    {
      // no base topic subscriptions
    }

    /*!
     * \brief Advertise all publishers.
     *
     * \param nh          Relay ROS node handle of the embedded sentinel.
     * \param nQueueDepth Maximum dsend queue depth.
     */
    virtual void advertisePublishers(ros::NodeHandle &nh, int nQueueDepth=1)
    {
      // no base topic publishers
    }

    /*!
     * \brief Advertise all server services.
     *
     * \param nh          Relay ROS node handle of the embedded sentinel.
     */
    virtual void advertiseServices(ros::NodeHandle &nh)
    {
      // no base node services
    }

    /*!
     * \brief Initialize client services.
     */
    virtual void clientServices(ros::NodeHandle &nh)
    {
      // no base client services
    }

    /*!
     * \@{
     *
     * \brief Relay node sentinel callback on dwell update.
     *
     * \param gei         Geofrenzy entitlement index.
     * \param isInFence   The current location is currently [not] inside a 
     *                    geofence.
     * \param data        Data specific to entitlement data type.
     */
    virtual void cbWatchForBreach(const GfEntitlementIndex gei,
                                  const bool               isInFence,
                                  const GfEntBaseBoolset   &data);

    virtual void cbWatchForBreach(const GfEntitlementIndex gei,
                                  const bool               isInFence,
                                  const GfEntBaseProfile   &data);

    virtual void cbWatchForBreach(const GfEntitlementIndex gei,
                                  const bool               isInFence,
                                  const GfEntBaseThreshold &data);
    /*! \@} */

    /*!
     * \brief Clear data.
     */
    void clear();
    
    /*!
     * \@{
     * \brief Attribute gets.
     */
    Mep mep() const { return m_mep; }

    GfClassIndex gci() const { return m_gci; }

    BreachTrigger triggerType() const  { return m_breachTrigger; }

    BreachAction actionCategory()const { return m_breachAction; }

    bool isInFence() const { return m_isInFence; }

    bool isInBreach() const { return m_isInBreach; }
    /*! \@} */

    /*!
     * \brief Set breach trigger type.
     *
     * May alter in-breach state.
     */
    BreachTrigger setTriggerType(const BreachTrigger trigger);

    /*!
     * \brief Set in-breach boolean state given the current geofence position.
     *
     * \param isInFence The current location is currently [not] inside a 
     *                  geofence.
     *
     * \reutrn Returns new in-breach state.
     */
    virtual bool setBreachState(const bool isInFence);

    /*!
     * \brief Entitlement of Interest check.
     *
     * \param gci         Geofrenzy class index of entitlement.
     * \param gei         Geofrenzy entitlement index.
     * \param entDataType Geofrenzy entitlement type enumeration.
     *
     * return Returns true if this sentinel has interest in watching for state
     * changes for an entitlement with the identifying pattern.
     * Returns false otherwise.
     */
    virtual bool eoiCheck(const GfClassIndex       gci,
                          const GfEntitlementIndex gei,
                          const GfEntDataType      entDataType) const;

    /*!
     * \brief Get the size (number of) entitlements of interest.
     *
     * \return Size.
     */
    size_t eoiSize() const;

    /*!
     * \brief Get the entitlement of interest at index.
     *
     * An assert is thrown if index is out-of-bounds.
     *
     * \param i   Index.
     *
     * \return Reference to constant Entitlement of Interest.
     */
    const EoI &eoiAt(const size_t i) const;

    /*!
     * \brief Find the first entitlement of interest that matches pattern.
     *
     * \param gei         Geofrenzy entitlement index.
     * \param entDataType Geofrenzy entitlement type enumeration.
     *
     * \return Returns index to EoI on success. Returns -1 on failure.
     */
    ssize_t eoiFind(const GfEntitlementIndex gei,
                    const GfEntDataType      entDataType) const;

    /*!
     * \brief Get Message Exchange Pattern name.
     *
     * \param mep Message Exchange Pattern enumeration.
     *
     * \return String name
     */
    static std::string mepName(const GfSentinel::Mep mep);

    /*!
     * \brief Get trigger type name.
     *
     * \param mep Trigger type enumeration.
     *
     * \return String name
     */
    static std::string triggerName(const GfSentinel::BreachTrigger trigger);

    /*!
     * \brief Get action category name.
     *
     * \param mep Action category enumeration.
     *
     * \return String name
     */
    static std::string actionName(const GfSentinel::BreachAction action);

    /*!
     * \brief Virtualize print function so that insertion operator overloading
     * works.
     *
     * \param os  Output stream.
     */
    virtual void print(std::ostream &os) const;

    /*!
     * \brief Stream insertion operator.
     *
     * \param os  Output stream.
     * \param obj Object to insert.
     *
     * \param Returns reference to stream.
     */
    friend std::ostream &operator<<(std::ostream &os, const GfSentinel &obj);

  protected:
    //
    // Sentinel meta-data
    //
    GfClassIndex        m_gci;          ///< geofrenzy portal server class index
    Mep                 m_mep;          ///< message exchange pattern
    BreachTrigger       m_breachTrigger; ///< breach trigger event type
    BreachAction        m_breachAction; ///< breach action category
    EoIList             m_listEoI;      ///< list of entitlements of interest

    //
    // Sentinel state
    //
    bool  m_isInFence;    ///< mobile device is [not] in a geofence
    bool  m_isInBreach;   ///< mobile device is [not] in breach of entitlements

    //
    // ROS nannied topic and service names
    //
    std::string m_topicIn;      ///< subscribed topic
    std::string m_topicOut;     ///< published topic
    std::string m_serviceIn;    ///< service in
    std::string m_serviceOut;   ///< service out

    //
    // ROS message interface
    //
    gf_ros::MapPublishers     m_publishers;     ///< topic publishers
    gf_ros::MapSubscriptions  m_subscriptions;  ///< topic subscriptions
    gf_ros::MapServices       m_services;       ///< advertise services
    gf_ros::MapClientServices m_clientServices; ///< client services

    /*!
     * \brief Get unsigned 64-bit value from parameter server.
     *
     * Note: no 64-bit types supported parameter server
     *
     * \param nh          Relay ROS node handle of the embedded sentinel.
     * \param paramName   Parameter name.
     * \param dftVal      Parameter default value.
     *
     * \return Parameter server value or default.
     */
    uint64_t paramU64(ros::NodeHandle   &nh,
                      const std::string &paramName,
                      const uint64_t    dftVal);

    /*!
     * \brief Get signed 64-bit value from parameter server.
     *
     * Note: no 64-bit types supported parameter server
     *
     * \param nh          Relay ROS node handle of the embedded sentinel.
     * \param paramName   Parameter name.
     * \param dftVal      Parameter default value.
     *
     * \return Parameter server value or default.
     */
    int64_t paramS64(ros::NodeHandle   &nh,
                     const std::string &paramName,
                     const int64_t     dftVal);

  }; // class GfSentinel


  // ---------------------------------------------------------------------------
  // GfSentinelCam Derived Class
  // ---------------------------------------------------------------------------

  /*!
   * \brief Camera sentinel class.
   *
   * This sentinel class monitors an image stream. When a breach occurs, the
   * output is a censored image. When the breach is remediated, the image
   * stream continues.
   */
  class GfSentinelCam : public GfSentinel
  {
  public:
    /*!
     * \brief Default constructor.
     */
    GfSentinelCam();

    /*!
     * \brief Destructor.
     */
    virtual ~GfSentinelCam();

    /*!
     * \brief Initialize all quasi-static properties.
     *
     * \param nh    Relay ROS node handle of the embedded sentinel.
     * \param gci   Geofrenzy class index of ROS portal server.
     */
    virtual void initProperties(ros::NodeHandle &nh, GfClassIndex gci);

    /*!
     * \brief Subscribe to all topics.
     *
     * Excluded are Geofrenzy dwell breach topics which are handle by the
     * sensor relay node.
     *
     * \param nh          Relay ROS node handle of the embedded sentinel.
     * \param nQueueDepth Maximum receive queue depth.
     */
    virtual void subscribeToTopics(ros::NodeHandle &nh, int nQueueDepth=5);

    /*!
     * \brief Advertise all publishers.
     *
     * \param nh          Relay ROS node handle of the embedded sentinel.
     * \param nQueueDepth Maximum dsend queue depth.
     */
    virtual void advertisePublishers(ros::NodeHandle &nh, int nQueueDepth=1);

    /*!
     * \brief Virtualize print function so that insertion operator overloading
     * works.
     *
     * \param os  Output stream.
     */
    virtual void print(std::ostream &os) const;

    /*!
     * \brief Stream insertion operator.
     *
     * \param os  Output stream.
     * \param obj Object to insert.
     *
     * \param Returns reference to stream.
     */
    friend std::ostream &operator<<(std::ostream &os, const GfSentinelCam &obj);

  protected:
    std::string         m_imgFilename;  ///< censored image filename
    sensor_msgs::Image  m_imgCensored;  ///< image to publish when camera is not
                                        ///< permitted

    /*!
     * \brief Open or make a censored image.
     *
     * \param nh  Relay ROS node handle of the embedded sentinel.
     */
    virtual void makeCensoredImage(ros::NodeHandle &nh);

    /*!
     * \brief Received subscribed image message callback.
     *
     * \brief img   Image message.
     */
    virtual void cbImage(const sensor_msgs::Image &img);
  }; // class GfSentinelCam


  // ---------------------------------------------------------------------------
  // GfSentinelStop Derived Class
  // ---------------------------------------------------------------------------

  /*!
   * \brief Stop all motion sentinel class.
   *
   * This sentinel class monitors a twist velocity stream. When a breach occurs,
   * the mobile device is stopped.
   */
  class GfSentinelStop : public GfSentinel
  {
  public:
    /*!
     * \brief Default constructor.
     */
    GfSentinelStop();

    /*!
     * \brief Destructor.
     */
    virtual ~GfSentinelStop();

    /*!
     * \brief Initialize all quasi-static properties.
     *
     * \param nh    Relay ROS node handle of the embedded sentinel.
     * \param gci   Geofrenzy class index of ROS portal server.
     */
    virtual void initProperties(ros::NodeHandle &nh, GfClassIndex gci);

    /*!
     * \brief Subscribe to all topics.
     *
     * Excluded are Geofrenzy dwell breach topics which are handle by the
     * sensor relay node.
     *
     * \param nh          Relay ROS node handle of the embedded sentinel.
     * \param nQueueDepth Maximum receive queue depth.
     */
    virtual void subscribeToTopics(ros::NodeHandle &nh, int nQueueDepth=5);

    /*!
     * \brief Advertise all publishers.
     *
     * \param nh          Relay ROS node handle of the embedded sentinel.
     * \param nQueueDepth Maximum dsend queue depth.
     */
    virtual void advertisePublishers(ros::NodeHandle &nh, int nQueueDepth=1);

    /*!
     * \brief Virtualize print function so that insertion operator overloading
     * works.
     *
     * \param os  Output stream.
     */
    virtual void print(std::ostream &os) const;

    /*!
     * \brief Stream insertion operator.
     *
     * \param os  Output stream.
     * \param obj Object to insert.
     *
     * \param Returns reference to stream.
     */
    friend std::ostream &operator<<(std::ostream         &os,
                                    const GfSentinelStop &obj);

  protected:
    geometry_msgs::Twist m_msgTwistStop;
    geometry_msgs::Twist m_msgTwistOut;

    /*!
     * \brief Received subscribed twist velocity message callback.
     *
     * \param msgTwist  ROS twist message.
     */
    virtual void cbVel(const geometry_msgs::Twist &msgTwist);

  }; // class GfSentinelStop


  // ---------------------------------------------------------------------------
  // GfSentinelMavRtl Derived Class
  // ---------------------------------------------------------------------------

  /*!
   * \brief UAS return-to-landing sentinel class .
   *
   * This sentinel class monitors a UAS using mavros. When a breach occurs,
   * UAS control is taken to force return to landing.
   */
  class GfSentinelMavRtl : public GfSentinel
  {
  public:
    /*!
     * \brief Geographic position base on WGS 84 ellipsoid.
     */
    struct GeoPos
    {
      double  m_latitude;   ///< + north of equator, - south (degrees)
      double  m_longitude;  ///< + east of prime meridian, - west (degrees)
      double  m_altitude;   ///< + above WGS 84 ellipsoid
    };

    /*!
     * \brief Default constructor.
     */
    GfSentinelMavRtl();

    /*!
     * \brief Destructor.
     */
    virtual ~GfSentinelMavRtl();

    /*!
     * \brief Initialize all quasi-static properties.
     *
     * \param nh    Relay ROS node handle of the embedded sentinel.
     * \param gci   Geofrenzy class index of ROS portal server.
     */
    virtual void initProperties(ros::NodeHandle &nh, GfClassIndex gci);

    /*!
     * \brief Subscribe to all topics.
     *
     * Excluded are Geofrenzy dwell breach topics which are handle by the
     * sensor relay node.
     *
     * \param nh          Relay ROS node handle of the embedded sentinel.
     * \param nQueueDepth Maximum receive queue depth.
     */
    virtual void subscribeToTopics(ros::NodeHandle &nh, int nQueueDepth=5);

    /*!
     * \brief Advertise all publishers.
     *
     * \param nh          Relay ROS node handle of the embedded sentinel.
     * \param nQueueDepth Maximum dsend queue depth.
     */
    virtual void advertisePublishers(ros::NodeHandle &nh, int nQueueDepth=1);

    /*!
     * \brief Initialize client services.
     */
    virtual void clientServices(ros::NodeHandle &nh);

    /*!
     * \brief Set in-breach boolean state given the current geofence position.
     *
     * \param isInFence The current location is currently [not] inside a 
     *                  geofence.
     *
     * \reutrn Returns new in-breach state.
     */
    virtual bool setBreachState(const bool isInFence);

    /*!
     * \@{
     *
     * \brief Relay node sentinel callback on dwell update.
     *
     * \param gei         Geofrenzy entitlement index.
     * \param isInFence   The current location is currently [not] inside a 
     *                    geofence.
     * \param data        Data specific to entitlement data type.
     */
    virtual void cbWatchForBreach(const GfEntitlementIndex gei,
                                  const bool               isInFence,
                                  const GfEntBaseBoolset   &data);

    virtual void cbWatchForBreach(const GfEntitlementIndex gei,
                                  const bool               isInFence,
                                  const GfEntBaseThreshold &data);
    /*! \@} */

    /*!
     * \brief Virtualize print function so that insertion operator overloading
     * works.
     *
     * \param os  Output stream.
     */
    virtual void print(std::ostream &os) const;

    /*!
     * \brief Stream insertion operator.
     *
     * \param os  Output stream.
     * \param obj Object to insert.
     *
     * \param Returns reference to stream.
     */
    friend std::ostream &operator<<(std::ostream &os,
                                    const GfSentinelMavRtl &obj);

    friend std::ostream &operator<<(std::ostream &os,
                                    const GfSentinelMavRtl::GeoPos &obj);

  protected:
    //
    // State
    //
    bool        m_hasLandingPos;  ///< UAS does [not] have a landing position
    bool        m_isLanding;      ///< UAS is [not] in the process of landing
    bool        m_isOnTheGround;  ///< UAS is [not] on the ground
    double      m_flightCeiling;  ///< absolute flight ceiling (meters)
    bool        m_isArmed;        ///< UAS is [not] armed
    std::string m_flightMode;     ///< UAS mode (e.g. MANUAL, AUTO-RTL, etc)
    GeoPos      m_posHome;        ///< home (landing) geographic position
    GeoPos      m_posCur;         ///< current geographic position

    //
    // Support topics
    //
    std::string m_topicState;     ///< state topic
    std::string m_topicExState;   ///< extended state topic
    std::string m_topicHomePos;   ///< home position topic
    std::string m_topicGlobalPos; ///< global position topic

    //
    // Support client services
    //
    std::string m_serviceLandNow;   ///< land immediately
    std::string m_serviceSetMode;   ///< set operational mode
    std::string m_serviceSetHome;   ///< set home geographic position
    
    /*!
     * \brief Received subscribed twist velocity message callback.
     *
     * \param msgTwistStamped   ROS message.
     */
    virtual void cbVel(const geometry_msgs::Twist &msgTwistStamped);

    virtual void cbState(const mavros_msgs::State &msgState);

    virtual void cbExState(const mavros_msgs::ExtendedState &msgExState);

    virtual void cbHomePos(const mavros_msgs::HomePosition &msgHomePos);

    virtual void cbGlobalPos(const sensor_msgs::NavSatFix &msgFix);

    virtual bool returnToHome();

    virtual bool landNow();

    virtual bool setOpMode();

    virtual bool setHomePos();

  }; // class GfSentinelMavRtl

} // namespace geofrenzy

#endif // _GF_SENTINEL_H
