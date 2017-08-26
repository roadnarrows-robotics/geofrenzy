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

#include <stdlib.h>
#include <stdio.h>

#include <string>
#include <vector>

#include "boost/assign.hpp"

#include "std_msgs/Header.h"

#include "gf_types.h"
#include "gf_ros.h"

using namespace std;
using namespace boost::assign;

namespace geofrenzy
{
  /*!
   * \breif Entitlement data type enumeration base on the entitlement base
   * string.
   */
  std::map<std::string, GfEntDataType> EntBaseToType = map_list_of
    ("",          GfEntDataTypeUndef)
    ("boolset",   GfEntDataTypeBoolset)
    ("color",     GfEntDataTypeColor)
    ("json",      GfEntDataTypeJson)
    ("profile",   GfEntDataTypeProfile)
    ("threshold", GfEntDataTypeThreshold);

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
                              const string   gf_ent_base)
    {
      stringstream  ss;
    
      ss << makeNodeName(NodeRootFenceServer, gf_class_idx)
          << "/geofrenzy/"
          << gf_ent_idx
          << "/dwell/"
          << gf_ent_base;
    
      return ss.str();
    }
    
    string makeDwellTopicName(const uint64_t      gf_class_idx,
                              const uint64_t      gf_ent_idx,
                              const GfEntDataType gf_ent_type)     
    {
      stringstream   ss;

      ss << makeNodeName(NodeRootFenceServer, gf_class_idx)
          << "/geofrenzy/"
          << gf_ent_idx
          << "/dwell/"
          << entTypeToBase(gf_ent_type);

      return ss.str();
    }

    GfEntDataType entBaseToType(const std::string gf_ent_base)
    {
      if( EntBaseToType.find(gf_ent_base) != EntBaseToType.end() )
      {
        return EntBaseToType[gf_ent_base];
      }
      else
      {
        return GfEntDataTypeUndef;
      }
    }

    std::string entTypeToBase(const GfEntDataType gf_ent_type)
    {
      std::map<std::string, GfEntDataType>::const_iterator iter;

      for(iter = EntBaseToType.begin(); iter != EntBaseToType.end(); ++iter)
      {
        if( iter->second == gf_ent_type )
        {
          return iter->first;
        }
      }

      return "";
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

    void splitRosPackagePath(vector<string> &paths)
    {
      char *s = getenv("ROS_PACKAGE_PATH");

      // environment variable not set
      if( s == NULL )
      {
        return;
      }

      string  pkgpath(s);

      splitSearchPath(pkgpath, paths);
    }

    void splitSearchPath(const string &searchPath, vector<string> &paths)
    {
      size_t  m = 0;
      size_t  n;

      while( (n = searchPath.find(":", m)) != string::npos )
      {
        paths.push_back(searchPath.substr(m, n-m));
        m = n + 1;
      }

      if( m < searchPath.length() )
      {
        paths.push_back(searchPath.substr(m));
      }
    }

  } // namespace gf_ros

} // namespace geofrenzy
