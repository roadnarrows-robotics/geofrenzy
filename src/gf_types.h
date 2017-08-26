////////////////////////////////////////////////////////////////////////////////
//
// Package:   Geofrenzy Robot ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/geofrenzy
//
// File:      gf_types.h
//
/*! \file
 *
 * \brief The Geofrenzy ROS types.
 *
 * \note This header serves as a holding place defining the types and semantics
 * of the Goefrenzy Portal information. Much changes are expected.
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

#ifndef _GF_TYPES_H
#define _GF_TYPES_H

#include <math.h>
#include <string>

#include "ros/ros.h"

namespace geofrenzy
{
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Geofrenzy Types
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  typedef int64_t GfClassIndex;         ///< Geofrenzy class index
  typedef int64_t GfEntitlementIndex;   ///< Geofrenzy entitlement index

  /*!
   * \breif Entitlement base data type enumeration.
   */
  enum GfEntDataType
  {
    GfEntDataTypeUndef,     // undefined or unknown
    GfEntDataTypeBoolset,   // boolean bit set
    GfEntDataTypeColor,     // red-green-blue-alpha
    GfEntDataTypeJson,      // json encoded string
    GfEntDataTypeProfile,   // profile number
    GfEntDataTypeThreshold  // threshold fpn triple
  };

  typedef uint32_t GfEntBaseBoolset;    ///< set of boolean bits base data type

  /*!
   * \brief Color attribute base data type.
   */
  struct GfEntBaseColor
  {
    uint8_t m_red;      ///< red 8-bit channel
    uint8_t m_green;    ///< green 8-bit channel
    uint8_t m_blue;     ///< blue 8-bit channel
    uint8_t m_alpha;    ///< alpha channel with 0 = fully transparent
  };

  typedef uint64_t GfEntBaseProfile;  ///< numeric profile base data type

  /*!
   * \brief Threshold base data type.
   */
  struct GfEntBaseThreshold
  {
    double  m_upper;    ///< upper threshold
    double  m_lower;    ///< lower threshold
    double  m_units;    ///< conversion, units, context, etc
  };


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Fixed Geofrenzy Class Indices
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  const GfClassIndex GciUndef          = 0;    ///< undefined
  const GfClassIndex GciGuestAcct      = 1;    ///< guest account
  const GfClassIndex GciGeoNetwork     = 2;    ///< Geofrenzy
  const GfClassIndex GciRoadNarrowsLLC = 168;  ///< RoadNarrows - all things ROS
  const GfClassIndex GciAcmeCorp       = 1920; ///< beep, beep


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Reserved Geofrenzy Entitlement Indices.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Undefined
   */
  const GfEntitlementIndex GeiUndef = 0;

  /*!
   * \brief Fence Color.
   *
   * Color is an attribute the may be used for augumented or virtual realities.
   *
   * Base Type: color
   */
  const GfEntitlementIndex GeiFenceColor = 205;

  /*!
   * \brief No exit.
   *
   * The robotic system must remain in the defined fence.
   *
   * An exit event should trigger an override action that preempts any
   * current user or autonomous behavior.
   *
   * A 'return to home' action is an example override.
   *
   * Base Type: boolset
   */
  const GfEntitlementIndex GeiNoExit = 208;

  /*!
   * \brief No cameras allowed.
   *
   * Any camera streams must be censored while within the defined fence.
   *
   * An enter event should trigger an override action that censors any
   * cameras. The censor action may be simply turning off the cameras or
   * overlaying the recordings with a fixed informational image.
   *
   * Base Type: boolset
   */
  const GfEntitlementIndex GeiNoCameras = 209;

  /*!
   * \brief No entry.
   *
   * The robotic system must not enter the defined fence.
   *
   * An entry event should trigger an override action that preempts any
   * current user or autonomous behavior.
   *
   * A 'make an immediate landing' or 'stop all movements' actions are example
   * overrides.
   *
   * Base Type: boolset
   */
  const GfEntitlementIndex GeiNoEntry = 210;

  /*!
   * \brief Flight altitudes.
   *
   * The UAS system must remain between these altitudes while in the defined
   * fence.
   *
   * An altitude violation event should trigger an override action that
   * preempts any current user or autonomous behavior.
   *
   * A 'return to home' action is an example override.
   *
   * Base Type: threshold
   */
  const GfEntitlementIndex GeiFlightAltitudes = 211;

  /*!
   * \brief Issuing authority.
   *
   * All entitlements associated with a fence have the weight of the issuing
   * authority (e.g FAA).
   *
   * Base Type: profile
   */
  const GfEntitlementIndex GeiIssuingAuthority = 218;

} // namespace geofrenzy

#endif // _GF_TYPES_H
