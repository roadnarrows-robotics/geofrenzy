////////////////////////////////////////////////////////////////////////////////
//
// Package:   Geofrenzy Robot ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/geofrenzy
//
// File:      gf_vcloud.cpp
//
/*! \file
 *
 * \brief The Geofrenzy ROS cloud virtual sensor node source.
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
 * MIT
 * 
 * EULA:
 * See EULA.md
 */
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>

//
// System
//
#include <string>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>

//
// Add-on SDKs
//
#include "boost/assign.hpp"
#include "boost/bind.hpp"

//
// ROS
//
#include "ros/ros.h"
#include "ros/console.h"

//
// ROS generated core messages
//
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"

//
// ROS generated Geofrenzy messages
//
#include "geofrenzy/Polygon64.h"
#include "geofrenzy/GfDistFeature.h"
#include "geofrenzy/GfDistFeatureCollection.h"
#include "geofrenzy/GfDwellColor.h"
#include "geofrenzy/GfDwellThreshold.h"

//
// Geofrenzy
//
#include "gf_ros.h"
#include "gf_math.h"


using namespace std;
using namespace boost::assign;
using namespace geofrenzy::gf_ros;
using namespace geofrenzy::gf_math;

#ifdef GF_MATH_UT
#undef GF_CLOUD_NODE_UT   ///< cloud node unit test (only if math ut enabled)  
#endif // GF_MATH_UT

namespace geofrenzy
{
  //----------------------------------------------------------------------------
  // vSensorCloud Class
  //----------------------------------------------------------------------------

  /*!
   * \brief Geofrenzy xyzrgba point cloud virtual sensor ROS node class.
   */
  class vSensorCloud
  {
    public:
      /*!
       * \brief Default initialization constructor.
       *
       * \param gf_class_idx  Associated geofrenzy class index for this node.
       * \param nh            ROS node handle.
       * \param hz            Node hertz rate.
       */
      vSensorCloud(ros::NodeHandle &nh, double hz, uint64_t gf_class_idx) :
          m_gfClassIdx(gf_class_idx), m_nh(nh), m_hz(hz)
      {
        ROS_DEBUG_STREAM("vSensorCloud gf_class_idx = " << m_gfClassIdx);

        m_nPublishCnt = 0;
      };
  
      /*!
       * \brief Destructor.
       */
      ~vSensorCloud()
      {
      }

      /*!
       * \brief (Re)Initialize vSensor.
       *
       * \return Returns true on success, false otherwise.
       */
      bool init()
      {
        int       val;

        // horizontal field of view parameter server values
        m_nh.param(ParamNameCloudHFoVMin, m_fHFoVMin, CloudHFoVMinDft);
        m_nh.param(ParamNameCloudHFoVMax, m_fHFoVMax, CloudHFoVMaxDft);

        // vertical field of view parameter server values
        m_nh.param(ParamNameCloudVFoVMin, m_fVFoVMin, CloudVFoVMinDft);
        m_nh.param(ParamNameCloudVFoVMax, m_fVFoVMax, CloudVFoVMaxDft);

        // resolution (note: no unsigned i/f to parameter server
        m_nh.param(ParamNameCloudWidth, val, CloudWidthDft);
        m_uWidth = (size_t)val;
        m_nh.param(ParamNameCloudHeight, val, CloudHeightDft);
        m_uHeight = (size_t)val;

        // output format
        m_nh.param(ParamNameCloudPublishFmt, m_ePublishFmt, CloudPublishFmtDft);

        initCloudMsgFmt(m_msgCloud);

        ROS_INFO_STREAM("Cloud vSensor:" << endl
            << "  resolution:     " << m_uWidth << "x" << m_uHeight << endl
            << "  horizontal FoV: " << "[" << degrees(m_fHFoVMin) << ", "
                                    << degrees(m_fHFoVMax) << "]" << endl
            << "  vertical FoV:   " << "[" << degrees(m_fVFoVMin) << ", "
                                    << degrees(m_fVFoVMax) << "]" << endl
            << "  output format:  " << m_ePublishFmt);

#ifdef GF_CLOUD_NODE_UT
        utInit();
#endif // GF_CLOUD_NODE_UT

        return true;
      }

      /*!
       * \brief Advertise all server services.
       */
      void advertiseServices()
      {
      }

      /*!
       * \brief Initialize client services.
       */
      void clientServices()
      {
        // No client services
      }

      /*!
       * \brief Advertise all publishers.
       *
       * \param nQueueDepth   Maximum queue depth.
       */
      void advertisePublishers(int nQueueDepth=2)
      {
        m_publishers[TopicNameCloud] =
          m_nh.advertise<sensor_msgs::PointCloud2>(TopicNameCloud, 1);
      }

      /*!
       * \brief Subscribe to all topics.
       *
       * \param nQueueDepth   Maximum queue depth.
       */
      void subscribeToTopics(int nQueueDepth=5)
      {
        string  strServerNode = makeNodeName(NodeRootFenceServer, m_gfClassIdx);
        string  strTopic = "/" + strServerNode + "/" + TopicNameFcDist;

        m_subscriptions[strTopic] = m_nh.subscribe(strTopic, 1,
                                              &vSensorCloud::cbFcDist,
                                              &(*this));
      }

      /*!
       * \brief Publish.
       *
       * Call in main loop.
       */
      void publish()
      {
        if( m_nPublishCnt > 0 )
        {
          EigenXYZRGBAList      points;

          scanScene(m_fHFoVMin, m_fHFoVMax, m_fVFoVMin, m_fVFoVMax,
                    m_uWidth, m_uHeight, m_scene, points, ScanOptionDft);

          updateCloudMsg(points, m_msgCloud);

          m_publishers[TopicNameCloud].publish(m_msgCloud);

          --m_nPublishCnt;
        }
      }

    protected:
      ros::NodeHandle &m_nh;            ///< node handle
      double          m_hz;             ///< cycle hertz
      uint64_t        m_gfClassIdx;     ///< geofrenzy class index

      // ROS services, publishers, subscriptions.
      MapServices       m_services;       ///< Geofrenzy server services
      MapClientServices m_clientServices; ///< Geofrenzy server client services
      MapPublishers     m_publishers;     ///< Geofrenzy server publishers
      MapSubscriptions  m_subscriptions;  ///< Geofrenzy server subscriptions

      // sensor properties
      double  m_fHFoVMin;
      double  m_fHFoVMax;
      double  m_fVFoVMin;
      double  m_fVFoVMax;
      size_t  m_uWidth;
      size_t  m_uHeight;

      // scene
      EigenScene  m_scene;

      // messaging processing 
      int                       m_nPublishCnt;  ///< publish counter
      int                       m_ePublishFmt;  ///< publish output format
      sensor_msgs::PointCloud2  m_msgCloud;     ///< point cloud message

      void utInit()
      {
#ifdef GF_CLOUD_NODE_UT
        EigenSceneObj         sceneObj;
        geofrenzy::Polygon64  polygon;
        EigenPoint3           offset1(2.0, 0.0, 0.0);
        EigenPoint3           offset2(7.0, 5.0, 0.0);
        EigenPoint3           offset3(4.0, -5.0, 0.0);
        EigenPoint3           offset4(1.0, 4.0, 0.0);
        EigenRGBA             color1 = FenceColorDft;
        EigenRGBA             color2(0.5, 0.5, 0.1, 0.5);
        EigenRGBA             color3(0.5, 0.0, 0.5, 0.5);
        EigenRGBA             color4(0.1, 0.5, 0.1, 0.5);
        double                scale  = 0.1;

        //
        // Object one
        //
        utMakeCannedPolygon(UtPolynumTee, offset1, scale, polygon);
        m_scene.push_back(sceneObj);
        createSceneObj(polygon, color1, 2.0, m_scene.back());

        //
        // Object two
        //
        polygon.points.clear();
        utMakeCannedPolygon(UtPolynumRectangle, offset2, scale, polygon);
        m_scene.push_back(sceneObj);
        createSceneObj(polygon, color2, 2.0, m_scene.back());

        //
        // Object three
        //
        polygon.points.clear();
        utMakeCannedPolygon(UtPolynumHexagon, offset3, scale, polygon);
        m_scene.push_back(sceneObj);
        createSceneObj(polygon, color3, 2.0, m_scene.back());

        //
        // Object four
        //
        polygon.points.clear();
        utMakeCannedPolygon(UtPolynumTriangle, offset4, 0.05, polygon);
        m_scene.push_back(sceneObj);
        createSceneObj(polygon, color4, 2.0, m_scene.back());
#endif // GF_CLOUD_NODE_UT
      }

      void initCloudMsgFmt(sensor_msgs::PointCloud2 &msg)
      {
        sensor_msgs::PointCloud2Modifier modifier(msg);

        switch( m_ePublishFmt )
        {
          case CloudFmtXYZ:
            modifier.setPointCloud2FieldsByString(1, "xyz");
            break;
          case CloudFmtXYZRGBA:
            modifier.setPointCloud2FieldsByString(2, "xyz", "rgba");
            break;
          case CloudFmtXYZRGB:
          default:
            modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
            break;
        }
      }

      void updateCloudMsg(const EigenXYZRGBAList   &points, 
                          sensor_msgs::PointCloud2 &msg)
      {
        sensor_msgs::PointCloud2Modifier modifier(msg);

        //modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float>   iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float>   iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float>   iter_z(msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg, "b");

        size_t  i;

        //msg.data.clear();

        for(i = 0;
            i < points.size() && i < msg.data.size();
            ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
        {
          *iter_x = (float)points[i][_X];
          *iter_y = (float)points[i][_Y];
          *iter_z = (float)points[i][_Z];

          *iter_r = (uint8_t)(points[i][_RED]   * 255.0);
          *iter_g = (uint8_t)(points[i][_GREEN] * 255.0);
          *iter_b = (uint8_t)(points[i][_BLUE]  * 255.0);
        }

        stampHeader(msg.header, msg.header.seq+1, "cloud");
      }

      /*!
       * \brief Distance feature collection subscribed callback.
       */
      void cbFcDist(const geofrenzy::GfDistFeatureCollection::ConstPtr &msg)
      {
#ifndef GF_CLOUD_NODE_UT
        ROS_DEBUG(TopicNameFcDist);

        EigenSceneObj sceneObj;
        size_t        i, j;

        m_scene.clear();

        for(i = 0; i < msg->features.size(); ++i)
        {
          const GfDistFeature &feat = msg->features[i]; 

          for(j = 0; j < feat.geometry.size(); ++j)
          {
            m_scene.push_back(sceneObj);
            createSceneObj(feat.geometry[j], FenceColorDft, 2.0,
                m_scene.back());
          }
        }

        if( m_scene.size() > 0 )
        {
          ++m_nPublishCnt;
        }
#endif // !GF_CLOUD_NODE_UT
      }
  };


} // namespace geofrenzy

int main(int argc, char *argv[])
{
  double    hz;

  // get command-line Geofrenzy class index
  uint64_t gfClassIdx = paramClassIndex(argc, argv);

  // make a unique node name from the command line class index argument
  std::string nodeName = makeNodeName(NodeRootVCloud, gfClassIdx);

  // 
  // Initialize the node. Parse the command line arguments and environment to
  // determine ROS options such as node name, namespace and remappings.
  // This call does not contact the master. This lets you use
  // ros::master::check() and other ROS functions after calling ros::init()
  // to check on the status of the master.
  //
  ros::init(argc, argv, nodeName);

  // actual ROS-given node name
  nodeName = ros::this_node::getName();

  //
  // A ctrl-c interrupt will stop attempts to connect to the ROS core.
  //
  ros::NodeHandle nh(nodeName);

  //
  // Parse node command-line private (and static) arguments.
  //
  nh.param("hz", hz, 10.0);   // node hertz

  //
  // Failed to connect.
  //
  if( !ros::master::check() )
  {
    // add optional non-ROS unit tests here, then simply exit.
    return 0;
  }

  ROS_INFO_STREAM(nodeName << ": ROS master running.");

  //
  // Create Cloud virtual sensorr node.
  //
  geofrenzy::vSensorCloud vSensor(nh, hz, gfClassIdx);

  //
  // Initialize sensor.
  //
  if( !vSensor.init() )
  {
    ROS_FATAL_STREAM(nodeName << ": Failed to initialize.");
    return 2;
  }

  ROS_INFO_STREAM(nodeName << ": Node initialized.");

  //
  // Advertise services.
  //
  vSensor.advertiseServices();

  ROS_INFO_STREAM(nodeName << ": Advertised services registered.");

  //
  // Advertise publishers.
  //
  vSensor.advertisePublishers();
  
  ROS_INFO_STREAM(nodeName << ": Advertised publishers registered.");
  
  //
  // Subscribed to topics.
  //
  vSensor.subscribeToTopics();
  
  ROS_INFO_STREAM(nodeName << ": Subscribed topics registered.");

  // set loop rate in Hertz
  ros::Rate loop_rate(hz);

  ROS_INFO_STREAM(nodeName << ": Ready.");

  //
  // Main loop.
  //
  while( ros::ok() )
  {
    // make any callbacks on pending ROS events
    ros::spinOnce(); 

    // publish all readied advertised topics
    vSensor.publish();

    // sleep to keep at loop rate
    loop_rate.sleep();
  }

  return 0;
}
