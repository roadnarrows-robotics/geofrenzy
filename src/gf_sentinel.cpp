////////////////////////////////////////////////////////////////////////////////
//
// Package:   Geofrenzy Robot ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/geofrenzy
//
// File:      gf_sentinel.cpp
//
/*! \file
 *
 * \brief The Geofrenzy sentinel base and derived built-in class
 * implementations.
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
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stddef.h>

#include <string>
#include <map>
#include <ostream>

// uncomment to disable assert()
// #define NDEBUG
#include <cassert>

#include "boost/assign.hpp"

//
// ROS
//
#include "ros/ros.h"
#include "ros/console.h"

//
// ROS generated core messages
//
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_srvs/Trigger.h"

//
// mavros
//
#include "mavros_msgs/ExtendedState.h"
#include "mavros_msgs/HomePosition.h"
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/CommandLong.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandHome.h"

//
// ROS generated Geofrenzy messages
//
#include "geofrenzy/GfDwellBoolset.h"
#include "geofrenzy/GfDwellProfile.h"
#include "geofrenzy/GfDwellThreshold.h"
#include "geofrenzy/GfDwellJson.h"

//
// OpenCV
//
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//
// Geofrenzy
//
#include "gf_types.h"
#include "gf_ros.h"
#include "gf_sentinel.h"

using namespace boost::assign;
using namespace geofrenzy;
using namespace geofrenzy::gf_ros;


// -----------------------------------------------------------------------------
// Local Private
// -----------------------------------------------------------------------------

/*!
 * \breif Message Exchange Pattern names.
 */
static std::map<std::string, GfSentinel::Mep> GfMepNames =
  map_list_of
    ("undef",             GfSentinel::MepUndef)
    ("subscribe-publish", GfSentinel::MepSubPub)
    ("subscribe-service", GfSentinel::MepSubSvc)
    ("service-service",   GfSentinel::MepSvcSvc)
    ("service",           GfSentinel::MepSvc)
    ("api callback",      GfSentinel::MepCall);

/*!
 * \breif Breach trigger type names.
 */
static std::map<std::string, GfSentinel::BreachTrigger> GfTriggerNames =
  map_list_of
    ("undef",     GfSentinel::BreachTriggerUndef)
    ("on-entry",  GfSentinel::BreachTriggerEntry)
    ("on-exit",   GfSentinel::BreachTriggerExit);

/*!
 * \breif Breach action category names.
 */
static std::map<std::string, GfSentinel::BreachAction> GfActionNames =
  map_list_of
    ("undef",             GfSentinel::BreachActionUndef)
    ("censor",            GfSentinel::BreachActionCensor)
    ("return-to-launch",  GfSentinel::BreachActionRtl)
    ("all-stop",          GfSentinel::BreachActionStop)
    ("limit-speed",       GfSentinel::BreachActionLimitSpeed);


// -----------------------------------------------------------------------------
// GfSentinel Base Class
// -----------------------------------------------------------------------------

GfSentinel::GfSentinel(std::string nameOfSentinel) : m_nameOf(nameOfSentinel)
{
  clear();
}

GfSentinel::~GfSentinel()
{
}

void GfSentinel::initProperties(ros::NodeHandle &nh, GfClassIndex gci)
{
  m_gci = gci;
}

void GfSentinel::cbWatchForBreach(const GfEntitlementIndex gei,
                                  const bool               isInFence,
                                  const GfEntBaseBoolset   &data)
{
  setBreachState(isInFence);
}

void GfSentinel::cbWatchForBreach(const GfEntitlementIndex gei,
                                  const bool               isInFence,
                                  const GfEntBaseProfile   &data)
{
  setBreachState(isInFence);
}

void GfSentinel::cbWatchForBreach(const GfEntitlementIndex gei,
                                  const bool               isInFence,
                                  const GfEntBaseThreshold &data)
{
  setBreachState(isInFence);
}

void GfSentinel::clear()
{
  m_gci           = GciUndef;
  m_mep           = MepUndef;
  m_breachTrigger = BreachTriggerUndef;
  m_breachAction  = BreachActionUndef;
  m_listEoI.clear();

  m_isInFence     = false;
  m_isInBreach    = false;
}

GfSentinel::BreachTrigger GfSentinel::setTriggerType(
                                        const GfSentinel::BreachTrigger trigger)
{
  m_breachTrigger = trigger;
  setBreachState(m_isInFence);
  return m_breachTrigger;
}

bool GfSentinel::setBreachState(const bool isInFence)
{
  m_isInFence = isInFence;

  if( m_isInFence && (m_breachTrigger == BreachTriggerEntry) )
  {
    m_isInBreach = true;
  }
  else if( !m_isInFence && (m_breachTrigger == BreachTriggerExit) )
  {
    m_isInBreach = true;
  }
  else
  {
    m_isInBreach = false;
  }

  return m_isInBreach;
}

bool GfSentinel::eoiCheck(const GfClassIndex       gci,
                          const GfEntitlementIndex gei,
                          const GfEntDataType      entDataType) const
{
  return (gci == m_gci) && (eoiFind(gei, entDataType) >= 0);
}

size_t GfSentinel::eoiSize() const
{
  return m_listEoI.size();
}

const GfSentinel::EoI &GfSentinel::eoiAt(const size_t i) const
{
  assert(i < m_listEoI.size());
  return m_listEoI[i];
}

ssize_t GfSentinel::eoiFind(const GfEntitlementIndex gei,
                            const GfEntDataType      entDataType) const
{
  for(size_t i = 0; i < m_listEoI.size(); ++i)
  {
    if( (m_listEoI[i].m_gei == gei) &&
        (m_listEoI[i].m_entDataType == entDataType) )
    {
      return i;
    }
  }
  return -1;
}

std::string GfSentinel::makeDwellTopicName(const EoI &eoi)
{
  std::string slash("/");

  return slash +
              gf_ros::makeDwellTopicName(m_gci, eoi.m_gei, eoi.m_entDataType);
}

uint64_t GfSentinel::paramU64(ros::NodeHandle   &nh,
                              const std::string &paramName,
                              const uint64_t    dftVal)
{
  int32_t val;
  int32_t dft = (int32_t)dftVal;

  nh.param(paramName, val, dft);

  return (uint64_t)val;
}

int64_t GfSentinel::paramS64(ros::NodeHandle   &nh, 
                             const std::string &paramName,
                             const int64_t    dftVal)
{
  int32_t val;
  int32_t dft = (int32_t)dftVal;

  nh.param(paramName, val, dft);

  return (int64_t)val;
}

std::string GfSentinel::mepName(const GfSentinel::Mep mep)
{
  std::map<std::string, GfSentinel::Mep>::const_iterator iter;

  for(iter = GfMepNames.begin(); iter != GfMepNames.end(); ++iter)
  {
    if( iter->second == mep )
    {
      return iter->first;
    }
  }

  return GfMepNames.begin()->first;
}

std::string GfSentinel::triggerName(const GfSentinel::BreachTrigger trigger)
{
  std::map<std::string, GfSentinel::BreachTrigger>::const_iterator iter;

  for(iter = GfTriggerNames.begin(); iter != GfTriggerNames.end(); ++iter)
  {
    if( iter->second == trigger )
    {
      return iter->first;
    }
  }

  return GfTriggerNames.begin()->first;
}

std::string GfSentinel::actionName(const GfSentinel::BreachAction action)
{
  std::map<std::string, GfSentinel::BreachAction>::const_iterator iter;

  for(iter = GfActionNames.begin(); iter != GfActionNames.end(); ++iter)
  {
    if( iter->second == action )
    {
      return iter->first;
    }
  }

  return GfActionNames.begin()->first;
}

void GfSentinel::print(std::ostream &os) const
{
  os  << "GfSentinel" << std::endl
      << "{" << std::endl;

  os  << "meta-data:" << std::endl
      << "  nameOf       = " << nameOf() << std::endl
      << "  gci          = " << gci() << std::endl
      << "  mep          = "
        << GfSentinel::mepName(mep())
        << "(" << mep() << ")" << std::endl
      << "  trigger      = "
        << GfSentinel::triggerName(triggerType())
        << "(" << triggerType() << ")" << std::endl
      << "  action       = "
        << GfSentinel::actionName(actionCategory())
        << "(" << actionCategory() << ")" << std::endl;

  os  << "entitlements-of-interest:" << std::endl
        << "  eoi[" << eoiSize() << "] =" << std::endl
        << "  {" << std::endl;
  for(size_t i = 0; i < eoiSize(); ++i)
  {
    const GfSentinel::EoI &eoi = eoiAt(i);
    os  << "    {" << std::endl
        << "      gei   = " << eoi.m_gei << std::endl
        << "      type  = "
          << entTypeToBase(eoi.m_entDataType)
          << "(" << eoi.m_entDataType << ")" << std::endl
        << "      topic = '" << eoi.m_topicDwell << "'" << std::endl
        << "    }" << std::endl;
  }
  os  << "  }" << std::endl;

  os  << "state:" << std::endl
      << "  isInFence  = " << isInFence() << std::endl
      << "  isInBreach = " << isInBreach() << std::endl;

  os  << "subscriptions:" << std::endl
        << "  topics[" << m_subscriptions.size() << "] =" << std::endl
        << "  {" << std::endl;
  for(MapSubscriptions::const_iterator iter = m_subscriptions.begin();
      iter != m_subscriptions.end();
      ++iter)
  {
    os << "    " << iter->first;
    if( iter->first == m_topicIn )
    {
      os << "(*)";
    }
    os << std::endl;
  }
  os  << "  }" << std::endl;

  os  << "publishers:" << std::endl
        << "  topics[" << m_publishers.size() << "] =" << std::endl
        << "  {" << std::endl;
  for(MapPublishers::const_iterator iter = m_publishers.begin();
      iter != m_publishers.end();
      ++iter)
  {
    os << "    " << iter->first;
    if( iter->first == m_topicOut )
    {
      os << "(*)";
    }
    os << std::endl;
  }
  os  << "  }" << std::endl;

  os  << "node-services:" << std::endl
        << "  services[" << m_services.size() << "] =" << std::endl
        << "  {" << std::endl;
  for(MapServices::const_iterator iter = m_services.begin();
      iter != m_services.end();
      ++iter)
  {
    os << "    " << iter->first;
    if( iter->first == m_serviceIn )
    {
      os << "(*)";
    }
    os << std::endl;
  }
  os  << "  }" << std::endl;

  os  << "client-services:" << std::endl
        << "  services[" << m_clientServices.size() << "] =" << std::endl
        << "  {" << std::endl;
  for(MapClientServices::const_iterator iter = m_clientServices.begin();
      iter != m_clientServices.end();
      ++iter)
  {
    os << "    " << iter->first;
    if( iter->first == m_serviceOut )
    {
      os << "(*)";
    }
    os << std::endl;
  }
  os  << "  }" << std::endl;

  os  << "}" << std::endl;
}

std::ostream &geofrenzy::operator<<(std::ostream &os, const GfSentinel &obj)
{
  obj.print(os);

  return os;
}


// -----------------------------------------------------------------------------
// GfSentinelCam Derived Class
// -----------------------------------------------------------------------------

GfSentinelCam::GfSentinelCam() : GfSentinel("GfSentinelCam")
{
}

GfSentinelCam::~GfSentinelCam()
{
}

void GfSentinelCam::initProperties(ros::NodeHandle &nh, GfClassIndex gci)
{
  GfSentinel::EoI eoi;

  m_gci           = gci;
  m_mep           = MepSubPub;
  m_breachTrigger = BreachTriggerEntry;
  m_breachAction  = BreachActionCensor;

  //
  // Nannied topics.
  //
  m_topicIn  = "watch/image_raw";
  m_topicOut = "geofrenzy/image_raw";

  //
  // Only watch for a bool dwell topic associated with camera entitlements.
  //
  eoi.m_gei         = paramS64(nh, ParamNameSrCamGei, GeiNoCameras);
  eoi.m_entDataType = GfEntDataTypeBoolset;
  eoi.m_topicDwell  = makeDwellTopicName(eoi);
  m_listEoI.push_back(eoi);

  // Turn local image into ros message to publish
  makeCensoredImage(nh);
}

void GfSentinelCam::makeCensoredImage(ros::NodeHandle &nh)
{
  std::string               filename;
  std::vector<std::string>  paths;

  // get any image file name from parameter server
  nh.param(ParamNameSrCensoredImg, filename);

  //
  // Filename not found - use default image name.
  //
  if( filename.empty() )
  {
    splitRosPackagePath(paths);

    for(size_t i = 0; i < paths.size(); ++i)
    {
      filename = paths[i] + "/geofrenzy/images/censored.png";
      if( access(filename.c_str(), R_OK) == 0 )
      {
        break;
      }
      else
      {
        filename.clear();
      }
    }
  }

  cv::Mat img;

  // load image from file
  if( !filename.empty() && (access(filename.c_str(), R_OK) == 0) )
  {
    m_imgFilename = filename;
    img = cv::imread(m_imgFilename, CV_LOAD_IMAGE_COLOR);
  }

  // create an image
  else
  {
    // VGA dark blue (B,G,R)
    m_imgFilename = "the dark blue yonder";
    img = cv::Mat(640, 480, CV_8UC3, cv::Scalar(64, 0, 0));
  }

  cv_bridge::CvImage  img_bridge;
  std_msgs::Header    header;

  img_bridge = cv_bridge::CvImage(header,
                                  sensor_msgs::image_encodings::RGB8,
                                  img);

  img_bridge.toImageMsg(m_imgCensored);
}

void GfSentinelCam::subscribeToTopics(ros::NodeHandle &nh, int nQueueDepth)
{
  m_subscriptions[m_topicIn] =
      nh.subscribe(m_topicIn, nQueueDepth, &GfSentinelCam::cbImage, &(*this));
}

void GfSentinelCam::advertisePublishers(ros::NodeHandle &nh, int nQueueDepth)
{
  m_publishers[m_topicOut] =
      nh.advertise<sensor_msgs::Image>(m_topicOut, nQueueDepth, true);
}

void GfSentinelCam::cbImage(const sensor_msgs::Image &img)
{
  // censor images
  if( m_isInBreach )
  {
    m_imgCensored.header = img.header;
    m_publishers[m_topicOut].publish(m_imgCensored);
  }
      
  // okay to publish images
  else
  {
    m_publishers[m_topicOut].publish(img);
  }
}

void GfSentinelCam::print(std::ostream &os) const
{
  os  << nameOf() << "::";
  GfSentinel::print(os);

  os  << "GfSentinelCam" << std::endl
      << "{" << std::endl
      << "  censorFilename = '" << m_imgFilename << "'" << std::endl
      << "}" << std::endl;
}

std::ostream &geofrenzy::operator<<(std::ostream &os, const GfSentinelCam &obj)
{
  obj.print(os);

  return os;
}


// -----------------------------------------------------------------------------
// GfSentinelStop Derived Class
// -----------------------------------------------------------------------------

GfSentinelStop::GfSentinelStop() : GfSentinel("GfSentinelStop")
{
}

GfSentinelStop::~GfSentinelStop()
{
}

void GfSentinelStop::initProperties(ros::NodeHandle &nh, GfClassIndex gci)
{
  GfSentinel::EoI eoi;

  m_gci           = gci;
  m_mep           = MepSubPub;
  m_breachTrigger = BreachTriggerEntry;
  m_breachAction  = BreachActionStop;

  //
  // Nannied topics.
  //
  m_topicIn  = "watch/cmd_vel";
  m_topicOut = "geofrenzy/cmd_vel";

  //
  // Only watch for a bool dwell topic associated with stop entitlements.
  //
  eoi.m_gei         = paramS64(nh, ParamNameSrStopGei, GeiNoEntry);
  eoi.m_entDataType = GfEntDataTypeBoolset;
  eoi.m_topicDwell  = makeDwellTopicName(eoi);
  m_listEoI.push_back(eoi);

  //
  // Messages set to zero on construction
  //
}

void GfSentinelStop::subscribeToTopics(ros::NodeHandle &nh, int nQueueDepth)
{
  m_subscriptions[m_topicIn] =
      nh.subscribe(m_topicIn, 1, &GfSentinelStop::cbVel, &(*this));
}

void GfSentinelStop::advertisePublishers(ros::NodeHandle &nh, int nQueueDepth)
{
  m_publishers[m_topicOut] =
      nh.advertise<geometry_msgs::Twist>(m_topicOut, 2, true);
}

void GfSentinelStop::cbVel(const geometry_msgs::Twist &msgTwist)
{
  if( !m_isInBreach )
  {
    m_msgTwistOut = m_msgTwistStop;
  }

  // move as will
  else
  {
    m_msgTwistOut = msgTwist;
  }
      
  m_publishers[m_topicOut].publish(m_msgTwistOut);
}

void GfSentinelStop::print(std::ostream &os) const
{
  os  << nameOf() << "::";
  GfSentinel::print(os);

  os  << "GfSentinelStop" << std::endl
      << "{" << std::endl
      << "  vel_out = " << std::endl
      << "  {" << std::endl
      << m_msgTwistOut
      << "  }" << std::endl
      << "}" << std::endl;
}

std::ostream &geofrenzy::operator<<(std::ostream &os, const GfSentinelStop &obj)
{
  obj.print(os);

  return os;
}


// -----------------------------------------------------------------------------
// GfSentinelMavRtl Derived Class
// -----------------------------------------------------------------------------

GfSentinelMavRtl::GfSentinelMavRtl() : GfSentinel("SentinelMavRtl")
{
}

GfSentinelMavRtl::~GfSentinelMavRtl()
{
}

void GfSentinelMavRtl::initProperties(ros::NodeHandle &nh, GfClassIndex gci)
{
  GfSentinel::EoI eoi;

  m_gci           = gci;
  m_mep           = MepSvc;
  m_breachTrigger = BreachTriggerExit;
  m_breachAction  = BreachActionRtl;

  //
  // Nannied topics and services
  //
  // Note:  Not need. When RTL is issued, the UAS takes control.
  //
  //m_topicIn     = "watch/setpoint_velocity/cmd_vel";
  //m_topicOut    = "geofrenzy/setpoint_velocity/cmd_vel";

  // force return-to-launch service
  m_serviceOut  = "/mavros/cmd/command";

  //
  // Support topics
  //
  m_topicState      = "/mavros/state";
  m_topicExState    = "/mavros/extended_state";
  m_topicHomePos    = "/mavros/home_position/home";
  m_topicGlobalPos  = "/mavros/global_position/global";

  //
  // Support services
  //
  m_serviceLandNow    = "/mavros/cmd/land";
  m_serviceSetMode    = "/mavros/set_mode";
  m_serviceSetHomePos = "/mavros/set_home";
 
  //
  // Watch for a bool dwell topic associated with RTL entitlement.
  //
  eoi.m_gei         = paramS64(nh, ParamNameSrRtlGei, GeiNoExit);
  eoi.m_entDataType = GfEntDataTypeBoolset;
  eoi.m_topicDwell  = makeDwellTopicName(eoi);
  m_listEoI.push_back(eoi);

  //
  // Watch for an altitude threshold dwell topic associated with RTL
  // entitlement.
  //
  eoi.m_gei         = paramS64(nh, ParamNameSrAltitudeGei, GeiFlightAltitudes);
  eoi.m_entDataType = GfEntDataTypeThreshold;
  eoi.m_topicDwell  = makeDwellTopicName(eoi);
  m_listEoI.push_back(eoi);

  //
  // Extended state
  //
  m_hasLandingPos = false;  // unknown landing (home) geo position
  m_hasCurrentPos = false;  // unknown currernt geo position
  m_isLanding     = false;  // not in process of landing
  m_isOnTheGround = true;   // on the ground
  m_relCeiling    = 0.0;    // unknown relative flight ceiling
  m_flightCeiling = 0.0;    // unknown absolute flight ceiling
  m_isArmed       = false;  // UAS is unarmed

  // user-configured auto set manual op mode boolean
  nh.param(ParamNameSrAutoManual, m_autoManual, false);
}

void GfSentinelMavRtl::subscribeToTopics(ros::NodeHandle &nh, int nQueueDepth)
{
  // not needed
  //m_subscriptions[m_topicIn] =
  //    nh.subscribe(m_topicIn, 1, &GfSentinelMavRtl::cbVel, &(*this));

  m_subscriptions[m_topicState] =
      nh.subscribe(m_topicState, nQueueDepth,
                  &GfSentinelMavRtl::cbState, &(*this));

  m_subscriptions[m_topicExState] =
      nh.subscribe(m_topicExState, nQueueDepth,
                  &GfSentinelMavRtl::cbExState, &(*this));

  m_subscriptions[m_topicHomePos] =
      nh.subscribe(m_topicHomePos, 1, &GfSentinelMavRtl::cbHomePos, &(*this));

  m_subscriptions[m_topicGlobalPos] =
    nh.subscribe(m_topicGlobalPos, 1, &GfSentinelMavRtl::cbGlobalPos, &(*this));
}

void GfSentinelMavRtl::advertisePublishers(ros::NodeHandle &nh, int nQueueDepth)
{
  // not needed
  //m_publishers[m_topicOut] =
  //  nh.advertise<geometry_msgs::Twist>(m_topicOut, 2, true);
}

void GfSentinelMavRtl::clientServices(ros::NodeHandle &nh)
{
  // autonomous return-to-launch
  m_clientServices[m_serviceOut] =
    nh.serviceClient<mavros_msgs::CommandLong>(m_serviceOut);

  // land immediately
  m_clientServices[m_serviceLandNow] =
    nh.serviceClient<mavros_msgs::CommandTOL>(m_serviceLandNow);

  // set operational mode
  m_clientServices[m_serviceSetMode] =
    nh.serviceClient<mavros_msgs::SetMode>(m_serviceSetMode);

  // set home geographic position
  m_clientServices[m_serviceSetHomePos] =
    nh.serviceClient<mavros_msgs::CommandHome>(m_serviceSetHomePos);
}

bool GfSentinelMavRtl::setBreachState(const bool isInFence)
{
  m_isInFence = isInFence;

  //
  // Cannot be breached if the UAS is on the ground or is disarmed.
  //
  if( m_isOnTheGround || !m_isArmed )
  {
    m_isLanding  = false;
    m_isInBreach = false;
  }

  //
  // Don't clear any breach condition until safely landed.
  //
  else if( m_isLanding )
  {
    ROS_INFO_STREAM(nameOf() << ": "
        << "In-Breach: UAS is in the process of returning to launch.");
    m_isInBreach = true;
  }

  //
  // Inside the fence and a breach is triggered on entry.
  //
  else if( m_isInFence && (m_breachTrigger == BreachTriggerEntry) )
  {
    ROS_INFO_STREAM(nameOf() << ": "
        << "In-Breach: UAS entered restricted geofence.");
    m_isInBreach = true;
  }

  //
  // Outside the fence and a breach triggered on exit.
  //
  else if( !m_isInFence && (m_breachTrigger == BreachTriggerExit) )
  {
    ROS_INFO_STREAM(nameOf() << ": "
        << "In-Breach: UAS exited geofence perimeter.");
    m_isInBreach = true;
  }

  //
  // Flying too high.
  //
  else if( (m_flightCeiling > 0.0) && m_hasCurrentPos &&
      (m_posCur.m_altitude > m_flightCeiling) )
  {
    ROS_INFO_STREAM(nameOf() << ": "
        << "In-Breach: UAS exceeded flight ceiling." << std::endl
        << "  Home altitude:    " << m_posHome.m_altitude << std::endl
        << "  Flight ceiling:   " <<  m_flightCeiling << std::endl
        << "  Current altitude: " << m_posCur.m_altitude << std::endl
        << "  ----------------  " << std::endl
        << "  Difference:       " << m_flightCeiling - m_posCur.m_altitude);

    m_isInBreach = true;
  }

  //
  // Good to go.
  //
  else
  {
    m_isInBreach = false;
  }

  return m_isInBreach;
}

void GfSentinelMavRtl::cbWatchForBreach(const GfEntitlementIndex gei,
                                        const bool               isInFence,
                                        const GfEntBaseBoolset   &data)
{
  //ROS_INFO_STREAM(nameOf() << ": "
  //    << "WatchForBreach(boolset): " << "isInFence = " << isInFence);

  setBreachState(isInFence);

  if( m_isInBreach && !m_isLanding )
  {
    m_isLanding = reqReturnToHome();
  }
}

void GfSentinelMavRtl::cbWatchForBreach(const GfEntitlementIndex gei,
                                        const bool               isInFence,
                                        const GfEntBaseThreshold &data)
{
  //ROS_INFO_STREAM(nameOf() << ": "
  //    << "WatchForBreach(threshold): "
  //    << "isInFence = " << isInFence
  //    << " threshold.upper = " << data.m_upper);

  m_relCeiling = data.m_upper >= MinRelCeiling? data.m_upper: MinRelCeiling;

  if( m_hasLandingPos )
  {
    m_flightCeiling = m_posHome.m_altitude + m_relCeiling;

    ROS_INFO_STREAM(nameOf() << ": "
        << "Flight ceiling: " << std::endl
        << "  Home altitude:  " << m_posHome.m_altitude << std::endl
        << "  Flight ceiling: " <<  m_flightCeiling);
  }

  setBreachState(isInFence);

  if( m_isInBreach && !m_isLanding )
  {
    m_isLanding = reqReturnToHome();
  }
}

void GfSentinelMavRtl::cbVel(const geometry_msgs::Twist &msgTwistStamped)
{
  // blacklist if UAS is in-breach
  if( !m_isInBreach )
  {
    m_publishers[m_topicOut].publish(msgTwistStamped);
  }
}

void GfSentinelMavRtl::cbState(const mavros_msgs::State &msgState)
{
  bool wasArmed = m_isArmed;

  m_isArmed    = msgState.armed;
  m_flightMode = msgState.mode;

  //
  // Transition from disarmed to armed.
  //
  if( !wasArmed && m_isArmed )
  {
    ROS_INFO_STREAM(nameOf() << ": UAS ARMED");

    //
    // The home position update is slow. Provisionaly use the current 
    // position when the drone is armed. When the next home position update
    // occurs, the home position and flight ceiling will be adjusted.
    //
    if( m_hasCurrentPos )
    {
      m_posHome.m_latitude  = m_posCur.m_latitude;
      m_posHome.m_longitude = m_posCur.m_longitude;
      m_posHome.m_altitude  = m_posCur.m_altitude;
      m_hasLandingPos       = true;

      //
      // Set absolute flight ceiling if relative ceiling is known.
      //
      if( m_relCeiling >= MinRelCeiling )
      {
        m_flightCeiling = m_posHome.m_altitude + m_relCeiling;

        ROS_INFO_STREAM(nameOf() << ": "
          << "Provisional flight ceiling: " << std::endl
          << "  Home altitude:  " << m_posHome.m_altitude << std::endl
          << "  Flight ceiling: " <<  m_flightCeiling);
      }
    }
  }

  //
  // Transition from armed to disarmed.
  //
  else if( wasArmed && !m_isArmed )
  {
    ROS_INFO_STREAM(nameOf() << ": UAS DISARMED");

    // if user-configured, try to return to manual operation mode 
    if( m_autoManual )
    {
      reqSetOpMode();
    }
  }
}

void GfSentinelMavRtl::cbExState(const mavros_msgs::ExtendedState &msgExState)
{
  switch( msgExState.landed_state )
  {
    case mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND:
      m_isOnTheGround = true;
      ROS_INFO_STREAM(nameOf() << ": UAS is on the ground.");
      break;
    case mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR:
      ROS_INFO_STREAM(nameOf() << ": UAS is in flight.");
      m_isOnTheGround = false;
      break;
    case mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED:
    default:
      break;
  }
}

void GfSentinelMavRtl::cbHomePos(const mavros_msgs::HomePosition &msgHomePos)
{
  //
  // Set home (RTL) position. Home position can only be trusted when the
  // UAS has been armed.
  //
  if( m_isArmed )
  {
    m_posHome.m_latitude  = msgHomePos.latitude;
    m_posHome.m_longitude = msgHomePos.longitude;
    m_posHome.m_altitude  = msgHomePos.altitude;
    m_hasLandingPos       = true;

    ROS_INFO_STREAM(nameOf() << ": Got Home RTL Position");
  }

  //
  // Home is really untrustworthy.
  //
  else
  {
    m_hasLandingPos = false;
  }
}

void GfSentinelMavRtl::cbGlobalPos(const sensor_msgs::NavSatFix &msgFix)
{
  m_posCur.m_latitude  = msgFix.latitude;
  m_posCur.m_longitude = msgFix.longitude;
  m_posCur.m_altitude  = msgFix.altitude;
  m_hasCurrentPos      = true;
}

bool GfSentinelMavRtl::reqReturnToHome()
{
  std::string               &nameSvc = m_serviceOut;
  mavros_msgs::CommandLong  svc;

  //
  // The drone is not armed.
  //
  if( !m_isArmed )
  {
    return false;
  }

  //
  // Already landing.
  //
  // Note: May need more tests.
  //
  else if( (m_flightMode == "AUTO.RTL") || (m_flightMode == "AUTO.LAND") )
  {
    return false;
  }

  ROS_INFO_STREAM(nameOf() << ": Return To Launch");

  svc.request.broadcast     = false;
  svc.request.command       = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
  svc.request.confirmation  = true;

  if( m_clientServices[nameSvc].call(svc) )
  {
    ROS_DEBUG_STREAM(nameOf() << ": " << nameSvc);
    return svc.response.success;
  }
  else
  {
    ROS_ERROR_STREAM(nameOf() << ": " << "Service " << nameSvc << " failed.");
    return false;
  }
}

bool GfSentinelMavRtl::reqLandNow()
{
  std::string             &nameSvc = m_serviceLandNow;
  mavros_msgs::CommandTOL svc;

  //
  // The drone is not armed.
  //
  if( !m_isArmed )
  {
    return false;
  }
  
  ROS_INFO_STREAM(nameOf() << ": Land Now");

  //
  // N.B. Current version mavros land command ignores all fields and
  //      simply lands a current position.
  //
  svc.request.min_pitch = 0.0; // only used by takeoff
  svc.request.yaw       = 0.0; // may fix later to home position yaw
  svc.request.latitude  = m_posHome.m_latitude;
  svc.request.longitude = m_posHome.m_longitude;
  svc.request.altitude  = m_posHome.m_altitude;

  if( m_clientServices[nameSvc].call(svc) )
  {
    ROS_DEBUG_STREAM(nameOf() << ": " << nameSvc);
    return svc.response.success;
  }
  else
  {
    ROS_ERROR_STREAM(nameOf() << ": " << "Service " << nameSvc << " failed.");
    return false;
  }
}

bool GfSentinelMavRtl::reqSetOpMode()
{
  std::string           &nameSvc = m_serviceSetMode;
  mavros_msgs::SetMode  svc;

  //
  // Do not send new operational mode if the UAS current operational mode
  // is unknowned or if the UAS is armed.
  //
  if( m_flightMode.empty() || m_isArmed )
  {
    return false;
  }

  svc.request.base_mode   = mavros_msgs::SetModeRequest::MAV_MODE_MANUAL_ARMED;
  svc.request.custom_mode = "MANUAL";

  if( m_clientServices[nameSvc].call(svc) )
  {
    ROS_DEBUG_STREAM(nameOf() << ": " << nameSvc);
    return svc.response.success;
  }
  else
  {
    ROS_ERROR_STREAM(nameOf() << ": " << "Service " << nameSvc << " failed.");
    return false;
  }
}

bool GfSentinelMavRtl::reqSetHomePos()
{
  std::string               &nameSvc = m_serviceSetHomePos;
  mavros_msgs::CommandHome  svc;

  if( !m_hasLandingPos )
  {
    ROS_ERROR_STREAM(nameOf() << ": "
        << nameSvc << ": No home position to set.");
    return false;
  }

  svc.request.latitude  = m_posHome.m_latitude;
  svc.request.longitude = m_posHome.m_longitude;
  svc.request.altitude  = m_posHome.m_altitude;

  if( m_clientServices[nameSvc].call(svc) )
  {
    ROS_DEBUG_STREAM(nameOf() << ": " << nameSvc);
    return svc.response.success;
  }
  else
  {
    ROS_ERROR_STREAM(nameOf() << ": " << "Service " << nameSvc << " failed.");
    return false;
  }
}

void GfSentinelMavRtl::print(std::ostream &os) const
{
  os  << nameOf() << "::";
  GfSentinel::print(os);

  os  << "GfSentinelMavRtl" << std::endl
      << "{" << std::endl
      << "state:" << std::endl
      << "  hasLandingPos  = " << m_hasLandingPos << std::endl
      << "  hasCurrentPos  = " << m_hasCurrentPos << std::endl
      << "  isLanding      = " << m_isLanding << std::endl
      << "  isOnTheGround  = " << m_isOnTheGround << std::endl
      << "  relCeiling     = " << m_relCeiling << std::endl
      << "  flightCeiling  = " << m_flightCeiling << std::endl
      << "  isArmed        = " << m_isArmed << std::endl
      << "  flightMode     = " << m_flightMode << std::endl
      << "  posHome        = " << m_posHome << std::endl
      << "  posCur         = " << m_posCur << std::endl
      << "  autoManual     = " << m_autoManual << std::endl
      << "}" << std::endl;
}

std::ostream &geofrenzy::operator<<(std::ostream &os,
                                    const GfSentinelMavRtl &obj)
{
  obj.print(os);

  return os;
}

std::ostream &geofrenzy::operator<<(std::ostream &os,
                                    const GfSentinelMavRtl::GeoPos &obj)
{
  os  << "(lat=" << obj.m_latitude  << ", "
      << "lon="  << obj.m_longitude << ", "
      << "alt="  << obj.m_altitude  << ")";

  return os;
}
