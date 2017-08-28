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
      BreachTriggerEnter, ///< breach is triggered on entering a geofence
      BreachTriggerExit   ///< breach is triggered on exiting a geofence
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
    virtual void subscribeToTopics(ros::NodeHandle &nh, int nQueueDepth=1);

    /*!
     * \brief Advertise all publishers.
     *
     * \param nh          Relay ROS node handle of the embedded sentinel.
     * \param nQueueDepth Maximum dsend queue depth.
     */
    virtual void advertisePublishers(ros::NodeHandle &nh, int nQueueDepth=1);

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
     * \brief Light-weight clearing of data.
     */
    void clear();
    
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
    virtual bool eoiMatch(const GfClassIndex       gci,
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
    const EoI &eoiAt(const size_t i);

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


    Mep mep() { return m_mep; }

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
    gf_ros::MapPublishers     m_publishers;     ///< topic publishers
    gf_ros::MapSubscriptions  m_subscriptions;  ///< topic subscriptions

    //
    // ROS nannied topic and service names
    //
    std::string m_topicIn;      ///< subscribed topic
    std::string m_topicOut;     ///< published topic
    std::string m_serviceIn;    ///< service in
    std::string m_serviceOut;   ///< service out


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
     * \brief Stream insertion operator.
     *
     * \param os  Output stream.
     * \param obj Object to insert.
     *
     * \param Returns reference to stream.
     */
    friend std::ostream &operator<<(std::ostream &os, const GfSentinel &obj);

  protected:
    sensor_msgs::Image m_imgCensored; ///< image to publish when camera is not
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
  };

} // namespace geofrenzy

#endif // _GF_SENTINEL_H
