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

#include <string>
#include <map>

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

using namespace geofrenzy;
using namespace geofrenzy::gf_ros;

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

void GfSentinel::cbWatchForBreach(bool isInFence)
{
  m_isInFence = isInFence;
}

void GfSentinel::clear()
{
  m_gci           = GciUndef;
  m_gei           = GeiUndef;
  m_entDataType   = GfEntDataTypeUndef;
  m_mep           = MepUndef;
  m_breachAction  = BreachActionUndef;
  m_isInFence     = false;
  m_isInBreach    = false;
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
  m_gci           = gci;
  m_entDataType   = GfEntDataTypeBoolset;
  m_mep           = MepSubPub;
  m_breachAction  = BreachActionCensor;

  //
  // Get "no cameras allowed" entitlement index.
  // 
  // Note: no 64-bit types in parameter server
  //
  int32_t val;
  int32_t dft = (int32_t)GeiNoCameras;

  nh.param(ParamNameSrCamGei, val, dft);

  m_gei = (GfEntitlementIndex)val;

  // topics
  m_topicIn     = "camera_in/image_raw";
  m_topicOut    = "/geofrenzy/image_raw";
  m_topicDwell  = makeDwellTopicName(m_gci, m_gei, m_entDataType);

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
      if( access(filename.c_str(), R_OK) )
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

  if( !filename.empty() && access(filename.c_str(), R_OK) )
  {
    img = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
  }
  else
  {
    // TODO make a darkblue image
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

void GfSentinelCam::cbWatchForBreach(bool isInFence)
{
  m_isInFence   = isInFence;
  m_isInBreach  = !m_isInFence;
}

void GfSentinelCam::cbImage(const sensor_msgs::Image &img)
{
  // censor images
  if( m_isInBreach )
  {
    // TODO fixup header
    m_publishers[m_topicOut].publish(m_imgCensored);
  }
      
  // okay to publish images
  else
  {
    m_publishers[m_topicOut].publish(img);
  }
}
