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
 * Apache 2.0
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
#include "geofrenzy/SetGeofenceAlt.h"

//
// ROS tranforms
//
#include "tf/transform_listener.h"

//
// Geofrenzy
//
#include "gf_ros.h"
#include "gf_math.h"
#include "gf_poly.h"
#include "gf_scene.h"


using namespace std;
using namespace boost::assign;
using namespace geofrenzy::gf_ros;
using namespace geofrenzy::gf_math;
using namespace geofrenzy::gf_scene;

//
// Unit Test Switch (only available if math ut is also enabled)  
//
#ifdef GF_SCENE_UT
#undef GF_VCLOUD_NODE_UT  ///< define to enable unit test
#else
#undef GF_VCLOUD_NODE_UT  ///< ut always disabled
#endif // GF_SCENE_UT

namespace geofrenzy
{
  static const map<int, string> NameOpMode = map_list_of
    (CloudOpModeSensor, "sensor")
    (CloudOpModeGrid,   "grid")
  ;

  static const map<int, string> NamePubFmt = map_list_of
    (CloudFmtXYZ,     "xyz")
    (CloudFmtXYZRGB,  "xyzrgb")
    (CloudFmtXYZRGBA, "xyzrgba")
  ;

  typedef std::map<int, string>::const_iterator name_iter;
  
  static const string NoName("?");

  static bool hasOpMode(int mode)
  {
    name_iter iter = NameOpMode.find(mode);

    return iter != NameOpMode.end();
  }

  static const string nameOfOpMode(int mode)
  {
    name_iter iter = NameOpMode.find(mode);

    return iter != NameOpMode.end()? iter->second: NoName;
  }

  static bool hasPubFmt(int fmt)
  {
    name_iter iter = NamePubFmt.find(fmt);

    return iter != NameOpMode.end();
  }

  static const string nameOfPubFmt(int fmt)
  {
    name_iter iter = NamePubFmt.find(fmt);

    return iter != NamePubFmt.end()? iter->second: NoName;
  }


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
#if NEWYEW
        m_pScanner    = NULL;
#endif // NEWYEW
      };
  
      /*!
       * \brief Destructor.
       */
      ~vSensorCloud()
      {
#if NEWYEW
        if( m_pScanner != NULL )
        {
          delete m_pScanner;
        }
#endif // NEWYEW
      }

      /*!
       * \brief (Re)Initialize vSensor.
       *
       * \return Returns true on success, false otherwise.
       */
      bool init()
      {
        string  name1, name2;   // working parameter names
        int     val;            // working value
        bool    tf;             // working boolean

        string  strInvalid("Invalid value: ");  // log string component
        string  strSetTo("set to default=");    // log string component

        m_uOptions = ScanOptionDft;

        //
        // Operation mode parameter.
        //
        name1 = ParamNameCloudOpMode;

        m_nh.param(name1, m_eOpMode, CloudOpModeDft);

        if( !hasOpMode(m_eOpMode) )
        {
          ROS_WARN_STREAM(strInvalid
              << name1 << "=" << m_eOpMode << ", "
              << strSetTo << CloudOpModeDft);
          m_eOpMode = CloudOpModeDft;
        }

        //
        // Horizontal field of view parameters.
        //
        name1 = ParamNameCloudHFoVMin;
        name2 = ParamNameCloudHFoVMax;

        m_nh.param(name1, m_fHFoVMin, CloudHFoVMinDft);
        m_nh.param(name2, m_fHFoVMax, CloudHFoVMaxDft);

        if( m_fHFoVMin < -M_PI )
        {
          ROS_WARN_STREAM(strInvalid
              << name1 << "=" << m_fHFoVMin << ", "
              << strSetTo << CloudHFoVMinDft);
          m_fHFoVMin = CloudHFoVMinDft;
        }
        if( m_fHFoVMax > M_PI )
        {
          ROS_WARN_STREAM(strInvalid
              << name2 << "=" << m_fHFoVMax << ", "
              << strSetTo << CloudHFoVMaxDft);
          m_fHFoVMax = CloudHFoVMaxDft;
        }
        if( m_fHFoVMin > m_fHFoVMax )
        {
          ROS_WARN_STREAM("Invalid range: "
              << name1 << "=" << m_fHFoVMin << " > "
              << name2 << "=" << m_fHFoVMax << ", "
              << "set to defaults=[" << CloudHFoVMinDft << ", "
              << CloudHFoVMaxDft << "]");
          m_fHFoVMin = CloudHFoVMinDft;
          m_fHFoVMax = CloudHFoVMaxDft;
        }

        //
        // Vertical field of view parameters.
        //
        name1 = ParamNameCloudVFoVMin;
        name2 = ParamNameCloudVFoVMax;

        m_nh.param(name1, m_fVFoVMin, CloudVFoVMinDft);
        m_nh.param(name2, m_fVFoVMax, CloudVFoVMaxDft);

        if( m_fVFoVMin < -0.0 )
        {
          ROS_WARN_STREAM(strInvalid
              << name1 << "=" << m_fVFoVMin << ", "
              << strSetTo << CloudVFoVMinDft);
          m_fVFoVMin = CloudVFoVMinDft;
        }
        if( m_fVFoVMax > M_PI )
        {
          ROS_WARN_STREAM(strInvalid
              << name2 << "=" << m_fVFoVMax << ", "
              << strSetTo << CloudVFoVMaxDft);
          m_fVFoVMax = CloudVFoVMaxDft;
        }
        if( m_fVFoVMin > m_fVFoVMax )
        {
          ROS_WARN_STREAM("Invalid range: "
              << name1 << "=" << m_fVFoVMin << " > "
              << name2 << "=" << m_fVFoVMax << ", "
              << "set to defaults=[" << CloudVFoVMinDft << ", "
              << CloudVFoVMaxDft << "]");
          m_fVFoVMin = CloudVFoVMinDft;
          m_fVFoVMax = CloudVFoVMaxDft;
        }

        //
        // Width resolution.
        // (note: no unsigned integer i/f to parameter server)
        //
        name1 = ParamNameCloudWidth;

        m_nh.param(name1, val, CloudWidthDft);

        if( val < 2 )
        {
          ROS_WARN_STREAM(strInvalid
              << name1 << "=" << val << ", "
              << strSetTo << CloudWidthDft);
          val = CloudWidthDft;
        }

        m_uWidth = (size_t)val;

        //
        // Height resolution.
        // (note: no unsigned integer i/f to parameter server)
        //
        name1 = ParamNameCloudHeight;

        m_nh.param(name1, val, CloudHeightDft);

        if( val < 2 )
        {
          ROS_WARN_STREAM(strInvalid
              << name1 << "=" << val << ", "
              << strSetTo << CloudHeightDft);
          val = CloudHeightDft;
        }

        m_uHeight = (size_t)val;

        //
        // Nearest Only Point Option.
        //
        name1 = ParamNameCloudNearestOnly;

        m_nh.param(name1, tf, CloudNearestOnlyDft);

        if( tf )
        {
          m_uOptions |= ScanOptionNearest;
        }
        else
        {
          m_uOptions &= ~ScanOptionNearest;
        }

        //
        // 2D Structured Output Option.
        //
        name1 = ParamNameCloud2D;

        m_nh.param(name1, tf, Cloud2DDft);

        if( tf )
        {
          m_uOptions |= (ScanOption2D | ScanOptionNearest);
        }
        else
        {
          m_uOptions &= ~ScanOption2D;
        }

        //
        // Grid size.
        //
        name1 = ParamNameCloudGridSize;

        m_nh.param(name1, m_fGridSize, CloudGridSizeDft);

        if( m_fGridSize <= 0.0 )
        {
          ROS_WARN_STREAM(strInvalid
              << name1 << "=" << m_fGridSize << ", "
              << strSetTo << CloudGridSizeDft);
          m_fGridSize = CloudGridSizeDft;
        }

        //
        // Published output format
        //
        name1 = ParamNameCloudPublishFmt;

        m_nh.param(name1, m_ePublishFmt, CloudPublishFmtDft);

        if( !hasPubFmt(m_ePublishFmt) )
        {
          ROS_WARN_STREAM(strInvalid
              << name1 << "=" << m_ePublishFmt << ", "
              << strSetTo << CloudPublishFmtDft);
          m_ePublishFmt = CloudPublishFmtDft;
        }

        //
        // Global frame and robot frame used for transformation.
        //
        m_nh.param(ParamNameGlobalFrame, m_globalFrame, GlobalFrameDft);
        m_nh.param(ParamNameRobotFrame, m_robotFrame, RobotFrameDft);

        initCloudMsgFmt(m_msgCloud);

        m_bCapFences      = false;
        m_fFenceAltitude  = 0.0;
        m_fFenceHeight    = GeofenceHeightDft;

#ifdef NEWYEW
        if( m_pScanner != NULL )
        {
          delete m_pScanner;
          m_pScanner = NULL;
        }

        switch( m_eOpMode )
        {
          case CloudOpModeSensor:
            //YEW m_pScanner = new SensorSceneScanner(..., m_uOptions);
            break;

          case CloudOpModeGrid:
            m_pScanner = new GridSceneScanner(m_fGridSize, m_uOptions);
            break;
        }
#endif // NEWYEW

        return true;
      }

      /*!
       * \brief Advertise all server services.
       */
      void advertiseServices()
      {
        std::string strSvc;

        strSvc = ServiceNameSetGeofenceAlt;
        m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &vSensorCloud::setGeofenceAlt,
                                          &(*this));
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

#ifdef GF_VCLOUD_NODE_UT
        m_subscriptions[strTopic] = m_nh.subscribe(strTopic, 1,
                                              &vSensorCloud::utFcDist,
                                              &(*this));
#else
        m_subscriptions[strTopic] = m_nh.subscribe(strTopic, 1,
                                              &vSensorCloud::cbFcDist,
                                              &(*this));
#endif // GF_VCLOUD_NODE_UT
      }

      /*!
       * \brief Publish.
       *
       * Call in main loop.
       */
      void publish()
      {
#if NEWYEW
        // no scaner
        if( m_pScanner == NULL )
        {
          return;
        }
#endif // NEWYEW

#ifdef GF_VCLOUD_NODE_UT
        // force publishing on every cycle
        if( m_nPublishCnt <= 0 )
        {
          utCreateFixedScene();
        }
#endif // GF_VCLOUD_NODE_UT

        if( m_nPublishCnt > 0 )
        {
          EigenXYZRGBAList      points;

#if NEWYEW
          ROS_INFO_STREAM("Scan " << m_scene.numOfFences() << " fences.");

          m_pScanner->scan(m_scene, points);
#else // the old you
          ROS_INFO_STREAM("Scan " << m_scene.size() << " fences.");

          if( m_eOpMode == CloudOpModeSensor )
          {
            scanScene(m_fHFoVMin, m_fHFoVMax, m_fVFoVMin, m_fVFoVMax,
            m_uWidth, m_uHeight, m_scene, points, m_uOptions);
          }
          else if( m_eOpMode == CloudOpModeGrid )
          {
            gridScene(m_fGridSize, m_scene, points, m_uOptions);
          }

#endif // NEWYEW

          ROS_INFO_STREAM("Update and publish cloud message.");

          updateCloudMsg(points, m_msgCloud);

          m_publishers[TopicNameCloud].publish(m_msgCloud);

          --m_nPublishCnt;
        }
      }

      /*!
       * \brief Log information about operational parameters to the ROS
       * logger (rosout).
       */
      void logOpParams()
      {
        ROS_INFO_STREAM(endl
            << "Geofrenzy vCloud:" << endl
            << "  Op Mode:        " << nameOfOpMode(m_eOpMode)
              << "(" << m_eOpMode << ")" << endl

            << "Sensor Mode Parameters:" << endl
            << "  resolution:     " << m_uWidth << "x" << m_uHeight << endl
            << "  horizontal FoV: " << "[" << degrees(m_fHFoVMin) << ", "
                                    << degrees(m_fHFoVMax) << "]" << endl
            << "  vertical FoV:   " << "[" << degrees(m_fVFoVMin) << ", "
                                    << degrees(m_fVFoVMax) << "]" << endl
            << "  nearest only:   "
              << (m_uOptions & ScanOptionNearest? "true": "false") << endl
            << "  2D structure:   "
              << (m_uOptions & ScanOption2D? "true": "false") << endl

            << "Grid Mode Parameters:" << endl
            << "  grid size:      " << m_fGridSize << endl

            << "Transform Parameters:" << endl
            << "  global frame:   " << m_globalFrame << endl
            << "  robot frame:    " << m_robotFrame << endl

            << "Publish Parameters:" << endl
            << "  output format:  " << nameOfPubFmt(m_ePublishFmt)
              << "(" << m_ePublishFmt << ")");
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
      int       m_eOpMode;    ///< operation mode
      double    m_fHFoVMin;   ///< min horizontal fov (radians)
      double    m_fHFoVMax;   ///< max horizontal fov (radians)
      double    m_fVFoVMin;   ///< min vertical fov (radians)
      double    m_fVFoVMax;   ///< max vertical fov (radians)
      size_t    m_uWidth;     ///< num of steps between min,max HFoV
      size_t    m_uHeight;    ///< num of steps between min,max VFoV
      double    m_fGridSize;  ///< grid size (meters)
      uint32_t  m_uOptions;   ///< scanning control options

      // scene global properties
      bool    m_bCapFences;     ///< [don't] cap geofences with floors/ceilings
      double  m_fFenceAltitude; ///< geofences base altitude (m)
      double  m_fFenceHeight;   ///< geofences height from base (m)

      // the dynamic scene
#ifdef NEWYEW
      GeofenceScene   m_scene;
      SceneScanner    *m_pScanner;
#else // the old you
      EigenScene  m_scene;      ///< scene of polygonal shapes with attributes
#endif // NEWYEW

      // messaging processing 
      int                       m_nPublishCnt;  ///< publish counter
      int                       m_ePublishFmt;  ///< publish output format
      sensor_msgs::PointCloud2  m_msgCloud;     ///< point cloud message

      //ROS transform listener
      tf::TransformListener m_tfListener;
      std::string           m_globalFrame;
      std::string           m_robotFrame;
      ros::Time             m_fixTime;

      /*!
       * \brief Initialize cloud message output format.
       *
       * \param msg PointCloud2 ROS message.
       */
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

      /*!
       * \brief Update cloud message with new scene points.
       *
       * \param points  Detected scene points.
       * \param msg     PointCloud2 ROS message.
       */
      void updateCloudMsg(const EigenXYZRGBAList   &points, 
                          sensor_msgs::PointCloud2 &msg)
      {
        sensor_msgs::PointCloud2Modifier modifier(msg);

        // resize cloud (is this light weight?)
        modifier.resize(points.size());

        // iterators
        sensor_msgs::PointCloud2Iterator<float>   iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float>   iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float>   iter_z(msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg, "b");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_a(msg, "a");

        //msg.data.clear();
 
        tf::StampedTransform transform;

#ifdef GF_VCLOUD_NODE_UT
        transform.getOrigin().setX(0.0);
        transform.getOrigin().setY(0.0);
#else
        // transform
        try{
          m_tfListener.waitForTransform(m_globalFrame, m_robotFrame, m_fixTime, ros::Duration(3.0));
          m_tfListener.lookupTransform(m_globalFrame, m_robotFrame, m_fixTime, transform);
        }
        catch(tf::TransformException ex){
          ROS_ERROR("Received exception trying to transform point from global frame "
                    "to robot frame: %s", ex.what());
        }
#endif // GF_VCLOUD_NODE_UT

        // Convert points to pointcloud
        for(size_t i = 0; i < points.size() && i < msg.data.size(); ++i)
        {
          *iter_x = (float)points[i][_X] + transform.getOrigin().getX();
          *iter_y = (float)points[i][_Y] + transform.getOrigin().getY();
          *iter_z = (float)points[i][_Z];

          ++iter_x; ++iter_y; ++iter_z;

          // include color
          if( (m_ePublishFmt == CloudFmtXYZRGB) ||
              (m_ePublishFmt == CloudFmtXYZRGBA) )
          {
            *iter_r = (uint8_t)(points[i][_XYZRED]   * Color24ChanMax);
            *iter_g = (uint8_t)(points[i][_XYZGREEN] * Color24ChanMax);
            *iter_b = (uint8_t)(points[i][_XYZBLUE]  * Color24ChanMax);

            ++iter_r; ++iter_g; ++iter_b;

            // include alpha
            if( m_ePublishFmt == CloudFmtXYZRGBA )
            {
              *iter_a = (uint8_t)(points[i][_XYZALPHA]  * Color24ChanMax);
              ++iter_a;
            }
          }
        }

        stampHeader(msg.header, msg.header.seq+1, "cloud");
      }

      /*!
       * \brief Distance feature collection subscribed callback.
       */
      void cbFcDist(const geofrenzy::GfDistFeatureCollection::ConstPtr &msg)
      {
        ROS_DEBUG(TopicNameFcDist);

        EigenSceneObj sceneObj;
        size_t        i, j, id;

        m_fixTime = msg->fix_time;
        m_scene.clear();

        for(i = 0, id = 0; i < msg->features.size(); ++i)
        {
          const GfDistFeature &feat = msg->features[i]; 

          for(j = 0; j < feat.geometry.size(); ++j, ++id)
          {
#ifdef NEWYEW
            m_scene.addFence(id, feat.geometry[j], FenceColorDft,
                             m_fFenceAltitude, m_fFenceHeight, m_bCapFences);
#else // the old you
            m_scene.push_back(sceneObj);
            createSceneObj(feat.geometry[j], FenceColorDft,
            m_fFenceAltitude, m_fFenceHeight, m_bCapFences,
            m_scene.back());
#endif // NEWYEW

          }
        }

#ifdef NEWYEW
        if( m_scene.numOfFences() > 0 )
#else // the old you
        if( m_scene.size() > 0 )
#endif // NEWYEW
        {
          ++m_nPublishCnt;
        }
      }

      /*!
       * \brief Set the geofence altitude callback.
       *
       * \param req   Service request.
       * \param rsp   Service response.
       *
       * \return Returns true on success, false on failure.
       */
      bool setGeofenceAlt(geofrenzy::SetGeofenceAlt::Request  &req,
                          geofrenzy::SetGeofenceAlt::Response &rsp)
      {
        ROS_DEBUG_STREAM(ServiceNameSetGeofenceAlt);

        m_fFenceAltitude = req.base;

        if( m_fFenceAltitude < 0.0 )
        {
          m_fFenceAltitude = 0.0;
        }

        m_fFenceHeight = req.height;

        if( m_fFenceHeight < m_fFenceAltitude + 0.1 )
        {
          m_fFenceHeight = m_fFenceAltitude + 0.1;
        }

        m_bCapFences = req.cap;

        return true;
      };

#ifdef GF_VCLOUD_NODE_UT
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Unit Test
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Unit test creation of fixed scene.
       */
      void utCreateFixedScene()
      {
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

        // tweaks
        double alt      = 0.0;  // fences base altitude (meters)
        double height   = 2.0;  // fences height (meters)
        bool   cap      = true; // do [not] cap fences with ceilings/floors

        m_scene.clear();

        //
        // Object one
        //
        utMakeCannedPolygon(UtPolynumTee, offset1, scale, polygon);
#ifdef NEWYEW
        m_scene.addFence(1, polygon, color1, alt, height, cap);
#else // the old you
        m_scene.push_back(sceneObj);
        createSceneObj(polygon, color1, alt, height, cap, m_scene.back());
#endif // NEWYEW

        //
        // Object two
        //
        polygon.points.clear();
        utMakeCannedPolygon(UtPolynumRectangle, offset2, scale, polygon);
#ifdef NEWYEW
        m_scene.addFence(2, polygon, color2, alt, height, cap);
#else // the old you
        m_scene.push_back(sceneObj);
        createSceneObj(polygon, color2, alt, height, cap, m_scene.back());
#endif // NEWYEW

        //
        // Object three
        //
        polygon.points.clear();
        utMakeCannedPolygon(UtPolynumHexagon, offset3, scale, polygon);
#ifdef NEWYEW
        m_scene.addFence(3, polygon, color3, alt+2.0, height, cap);
#else // the old you
        m_scene.push_back(sceneObj);
        createSceneObj(polygon, color3, alt, height, cap, m_scene.back());
#endif // NEWYEW

        //
        // Object four
        //
        polygon.points.clear();
        utMakeCannedPolygon(UtPolynumTriangle, offset4, 0.05, polygon);
#ifdef NEWYEW
        m_scene.addFence(4, polygon, color4, alt, height, cap);
#else // the old you
        m_scene.push_back(sceneObj);
        createSceneObj(polygon, color4, alt, height, cap, m_scene.back());
#endif // NEWYEW

        ++m_nPublishCnt;
      }

      /*!
       * \brief Unit test distance feature collection subscribed callback.
       */
      void utFcDist(const geofrenzy::GfDistFeatureCollection::ConstPtr &msg)
      {
        // ignore message, create fixed scene
        utCreateFixedScene();
      }
#endif // GF_VCLOUD_NODE_UT
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

  vSensor.logOpParams();

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
