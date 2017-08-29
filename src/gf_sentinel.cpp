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

//
// mavros
//

//
// ROS generated Geofrenzy messages
//
#include "geofrenzy/GfDwellBoolset.h"

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
    ("return-to-landing", GfSentinel::BreachActionRtl)
    ("all-stop",          GfSentinel::BreachActionStop)
    ("limit-speed",       GfSentinel::BreachActionLimitSpeed);


// -----------------------------------------------------------------------------
// GfSentinel Base Class
// -----------------------------------------------------------------------------

GfSentinel::GfSentinel()
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

void GfSentinel::subscribeToTopics(ros::NodeHandle &nh, int nQueueDepth)
{
}

void GfSentinel::advertisePublishers(ros::NodeHandle &nh, int nQueueDepth)
{
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
      << "{" << std::endl
      << "  gci          = " << gci() << std::endl
      << "  mep          = "
        << GfSentinel::mepName(mep())
        << "(" << mep() << ")" << std::endl
      << "  trigger      = "
        << GfSentinel::triggerName(triggerType())
        << "(" << triggerType() << ")" << std::endl
      << "  action       = "
        << GfSentinel::actionName(actionCategory())
        << "(" << actionCategory() << ")" << std::endl
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
  os  << "  isInFence  = " << isInFence() << std::endl
      << "  isInBreach = " << isInBreach() << std::endl;
  os  << "  topicIn    = '" << m_topicIn << "'" << std::endl
      << "  topicOut   = '" << m_topicOut << "'" << std::endl
      << "  serviceIn  = '" << m_serviceIn << "'" << std::endl
      << "  serviceOut = '" << m_serviceOut << "'" << std::endl;
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

GfSentinelCam::GfSentinelCam()
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
  m_topicIn  = "watch//image_raw";
  m_topicOut = "geofrenzy/image_raw";

  //
  // Only watch for a bool dwell topic associated with camera entitlements.
  //
  eoi.m_gei         = paramS64(nh, ParamNameSrCamGei, GeiNoCameras);
  eoi.m_entDataType = GfEntDataTypeBoolset;
  eoi.m_topicDwell  = makeDwellTopicName(m_gci, eoi.m_gei, eoi.m_entDataType);
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
  m_subscriptions[m_topicIn] = nh.subscribe(m_topicIn,
                                            nQueueDepth,
                                            &GfSentinelCam::cbImage,
                                            &(*this));
}

void GfSentinelCam::advertisePublishers(ros::NodeHandle &nh, int nQueueDepth)
{
  m_publishers[m_topicOut] = nh.advertise<sensor_msgs::Image>(m_topicOut,
                                                              nQueueDepth,
                                                              true);
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
  os  << "GfSentinelCam::";
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

GfSentinelStop::GfSentinelStop()
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
  eoi.m_topicDwell  = makeDwellTopicName(m_gci, eoi.m_gei, eoi.m_entDataType);
  m_listEoI.push_back(eoi);

  //
  // Messages set to zero on construction
  //
}

void GfSentinelStop::subscribeToTopics(ros::NodeHandle &nh, int nQueueDepth)
{
  m_subscriptions[m_topicIn] = nh.subscribe(m_topicIn,
                                            1,
                                            &GfSentinelStop::cbVel,
                                            &(*this));
}

void GfSentinelStop::advertisePublishers(ros::NodeHandle &nh, int nQueueDepth)
{
  m_publishers[m_topicOut] = nh.advertise<geometry_msgs::Twist>(m_topicOut,
                                                                2,
                                                                true);
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
  os  << "GfSentinelStop::";
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

GfSentinelMavRtl::GfSentinelMavRtl()
{
}

GfSentinelMavRtl::~GfSentinelMavRtl()
{
}

void GfSentinelMavRtl::initProperties(ros::NodeHandle &nh, GfClassIndex gci)
{
  GfSentinel::EoI eoi;

  m_gci           = gci;
  m_mep           = MepSubSvc;
  m_breachTrigger = BreachTriggerExit;
  m_breachAction  = BreachActionRtl;

  //
  // Nannied topics
  //
  m_topicIn     = "watch/setpoint_velocity/cmd_vel";
  m_topicOut    = "geofrenzy/setpoint_velocity/cmd_vel";

  //
  // Support topics
  //
  m_topicHomePos    = "/mavros/home_position/home";
  m_topicGlobalPos  = "/mavros/global_position/global";

  // force return-to-landing service
  m_serviceOut  = "/mavros/cmd/land";

  m_clientServices[m_serviceOut] =
    nh.serviceClient<mavros_msgs::CommandTOL>(m_serviceOut);

  //
  // Watch for a bool dwell topic associated with rtl entitlement.
  //
  eoi.m_gei         = paramS64(nh, ParamNameSrRtlGei, GeiNoExit);
  eoi.m_entDataType = GfEntDataTypeBoolset;
  eoi.m_topicDwell  = makeDwellTopicName(m_gci, eoi.m_gei, eoi.m_entDataType);
  m_listEoI.push_back(eoi);

  //
  // Watch for an altitude threshold dwell topic associated with rtl
  // entitlement.
  //
  eoi.m_gei         = paramS64(nh, ParamNameSrAltitudeGei, GeiFlightAltitudes);
  eoi.m_entDataType = GfEntDataTypeThreshold;
  eoi.m_topicDwell  = makeDwellTopicName(m_gci, eoi.m_gei, eoi.m_entDataType);
  m_listEoI.push_back(eoi);

  //
  // Extended state
  //
  m_hasLandingPos = false;
  m_isLanding     = false;
}

void GfSentinelMavRtl::subscribeToTopics(ros::NodeHandle &nh, int nQueueDepth)
{
  m_subscriptions[m_topicIn] = nh.subscribe(m_topicIn,
                                            1,
                                            &GfSentinelMavRtl::cbVel,
                                            &(*this));

  m_subscriptions[m_topicHomePos] = nh.subscribe(
                                            m_topicHomePos,
                                            1,
                                            &GfSentinelMavRtl::cbHomePos,
                                            &(*this));

  m_subscriptions[m_topicGlobalPos] = nh.subscribe(
                                            m_topicGlobalPos,
                                            1,
                                            &GfSentinelMavRtl::cbGlobalPos,
                                            &(*this));
}

void GfSentinelMavRtl::advertisePublishers(ros::NodeHandle &nh, int nQueueDepth)
{
  m_publishers[m_topicOut] = nh.advertise<geometry_msgs::Twist>(m_topicOut,
                                                                2,
                                                                true);
}

bool GfSentinelMavRtl::setBreachState(const bool isInFence)
{
  m_isInFence = isInFence;

  // don't clear any breach until safely landed within valid area
  if( m_isLanding )
  {
    m_isInBreach = true;
  }
  else if( m_isInFence && (m_breachTrigger == BreachTriggerEntry) )
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

void GfSentinelMavRtl::cbWatchForBreach(const GfEntitlementIndex gei,
                                        const bool               isInFence,
                                        const GfEntBaseBoolset   &data)
{
  setBreachState(isInFence);

  if( m_isInBreach )
  {
    if( !m_isLanding )
    {
      m_isLanding = returnToHome();
    }
  }
}

void GfSentinelMavRtl::cbWatchForBreach(const GfEntitlementIndex gei,
                                        const bool               isInFence,
                                        const GfEntBaseThreshold &data)
{
  setBreachState(isInFence);

  if( m_isInBreach )
  {
    if( !m_isLanding )
    {
      m_isLanding = returnToHome();
    }
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

void GfSentinelMavRtl::cbHomePos(const mavros_msgs::HomePosition &msgHomePos)
{
  m_posHome.m_latitude  = msgHomePos.latitude;
  m_posHome.m_longitude = msgHomePos.longitude;
  m_posHome.m_altitude  = msgHomePos.altitude;

  m_hasLandingPos = true;
}
void GfSentinelMavRtl::cbGlobalPos(const sensor_msgs::NavSatFix &msgFix)
{
  m_posCur.m_latitude  = msgFix.latitude;
  m_posCur.m_longitude = msgFix.longitude;
  m_posCur.m_altitude  = msgFix.altitude;
}

bool GfSentinelMavRtl::returnToHome()
{
  m_svcTOL.request.min_pitch = 0.0; // only used by takeoff
  m_svcTOL.request.yaw       = 0.0; // may fix later to home position yaw
  m_svcTOL.request.latitude  = m_posHome.m_latitude;
  m_svcTOL.request.longitude = m_posHome.m_longitude;
  m_svcTOL.request.altitude  = m_posHome.m_altitude;

  if( !m_hasLandingPos )
  {
    ROS_ERROR("No landing position known - cannot return!!!");
    return false;
  }

  else if( m_clientServices[m_serviceOut].call(m_svcTOL) )
  {
    ROS_DEBUG("RTL");
    return m_svcTOL.response.success;
  }
  else
  {
    ROS_ERROR("Failed RTL.");
    return false;
  }
}

bool GfSentinelMavRtl::onTheGround()
{
  return false;
}

void GfSentinelMavRtl::print(std::ostream &os) const
{
  os  << "GfSentinelMavRtl::";
  GfSentinel::print(os);

  os  << "GfSentinelMavRtl" << std::endl
      << "{" << std::endl
      << "}" << std::endl;
}

std::ostream &geofrenzy::operator<<(std::ostream           &os,
                                    const GfSentinelMavRtl &obj)
{
  obj.print(os);

  return os;
}
