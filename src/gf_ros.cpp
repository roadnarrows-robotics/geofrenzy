////////////////////////////////////////////////////////////////////////////////
//
// Package:   Geofrenzy Robot ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/geofrenzy
//
// File:      gf_ros.cpp
//
/*! \file
 *
 * \brief The Geofrenzy ROS top-level interface definitions.
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

#include <stdio.h>
#include <string>

#include "std_msgs/Header.h"

#include "gf_types.h"
#include "gf_ros.h"

using namespace std;

namespace geofrenzy
{
  namespace gf_ros
  {
    uint64_t paramClassIndex(int argc, char *argv[], uint64_t dft)
    {
      string         strArgLval("_gf_class_idx:=");
      string         strArgRval;
      long long unsigned  class_idx = dft;
    
      for(int i = 1; i < argc; ++i)
      {
        string strArg = argv[i];
    
        if( strArg.find(strArgLval) != string::npos )
        {
          strArgRval = strArg.substr(strArgLval.length());
    
          if( sscanf(strArgRval.c_str(), "%llu", &class_idx) != 1 )
          {
            // no ros logging available yet
            fprintf(stderr,
                "Warning: '%s' is not a valid Geofrenzy class index.",
                strArg.c_str());
          }
        }
      }
    
      return (uint64_t)class_idx;
    }
    
    string makeNodeName(const string strRoot, const uint64_t gf_class_idx)
    {
      stringstream  ss;
    
      ss << strRoot << "_" << gf_class_idx;
    
      return ss.str();
    }
      
    string makeDwellTopicName(const uint64_t gf_class_idx,
                              const uint64_t gf_ent_idx,
                              const string   gf_ent_subtype)
    {
      stringstream  ss;
    
      ss << makeNodeName(NodeRootFenceServer, gf_class_idx)
          << "/geofrenzy/"
          << gf_ent_idx
          << "/dwell/"
          << gf_ent_subtype;
    
      return ss.str();
    }
    
    void stampHeader(std_msgs::Header &header,
                     const int32_t    nSeqNum,
                     const string     strFrameId)
    {
      header.seq      = nSeqNum;
      header.stamp    = ros::Time::now();
      header.frame_id = strFrameId;
    }

    void stampHeader(std_msgs::Header &header,
                     const ros::Time  &stamp,
                     const int32_t    nSeqNum,
                     const string     strFrameId)
    {
      header.seq      = nSeqNum;
      header.stamp    = ros::Time::now();
      header.frame_id = strFrameId;
    }
  } // namespace gf_ros
} // namespace geofrenzy
