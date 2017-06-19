// GEOFRENZY FILE HEADER HERE

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

//
// ROS generated Geofrenzy messages
//
#include "geofrenzy/Polygon64.h"
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

namespace geofrenzy
{
  //----------------------------------------------------------------------------
  // vSensorRGBD Class
  //----------------------------------------------------------------------------

  /*!
   * \brief Geofrenzy Red-Green-Blue-Depth virtual sensor ROS node class.
   */
  class vSensorRGBD
  {
    public:
      /*!
       * \brief Default initialization constructor.
       *
       * \param gf_class_idx  Associated geofrenzy class index for this node.
       * \param nh            ROS node handle.
       * \param hz            Node hertz rate.
       */
      vSensorRGBD(ros::NodeHandle &nh, double hz, uint64_t gf_class_idx) :
          m_gfClass(gf_class_idx), m_nh(nh), m_hz(hz)
      {
        ROS_DEBUG_STREAM("vSensorRGBD gf_class_idx = " << m_gfClass);

        m_nPublishCnt = 0;
      };
  
      /*!
       * \brief Destructor.
       */
      ~vSensorRGBD()
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
        m_nh.param(ParamNameRGBDHFoVMin, m_fHFoVMin, RGBDHFoVMinDft);
        m_nh.param(ParamNameRGBDHFoVMax, m_fHFoVMax, RGBDHFoVMaxDft);

        // vertical field of view parameter server values
        m_nh.param(ParamNameRGBDVFoVMin, m_fVFoVMin, RGBDVFoVMinDft);
        m_nh.param(ParamNameRGBDVFoVMax, m_fVFoVMax, RGBDVFoVMaxDft);

        // resolution (note: no unsigned i/f to parameter server
        m_nh.param(ParamNameRGBDWidth, val, RGBDWidthDft);
        m_uWidth = (size_t)val;
        m_nh.param(ParamNameRGBDHeight, val, RGBDHeightDft);
        m_uHeight = (size_t)val;

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
      }

      /*!
       * \brief Subscribe to all topics.
       *
       * \param nQueueDepth   Maximum queue depth.
       */
      void subscribeToTopics(int nQueueDepth=5)
      {
      }

      /*!
       * \brief Publish.
       *
       * Call in main loop.
       */
      void publish()
      {
      }

    protected:
      ros::NodeHandle &m_nh;            ///< node handle
      double          m_hz;             ///< cycle hertz
      uint64_t        m_gfClass;        ///< geofrenzy class index

      // sensor properties
      double  m_fHFoVMin;
      double  m_fHFoVMax;
      double  m_fVFoVMin;
      double  m_fVFoVMax;
      size_t  m_uWidth;
      size_t  m_uHeight;

      // messaging processing overhead
      int       m_nPublishCnt;  ///< publish counter
  };


} // namespace geofrenzy

int main(int argc, char *argv[])
{
  geofrenzy::Polygon64  polygon;
  EigenPoint3           offset(10,0,0);

  //utMakeCannedPolygon(UtPolynumTriangle, offset, polygon);
  utMakeCannedPolygon(UtPolynumTee, offset, polygon);
  utScanPolygon(polygon);

  double    hz;

  // get command-line Geofrenzy class index
  uint64_t gfClassIdx = paramClassIndex(argc, argv);

  // make a unique node name from the command line class index argument
  std::string nodeName = makeNodeName(NodeRootRGBDVSensor, gfClassIdx);

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
  nh.param("hz", hz, 10.0);   // node hertz FenceServerrate

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
  // Create RGBD virtual sensorr node.
  //
  geofrenzy::vSensorRGBD vSensor(nh, hz, gfClassIdx);

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
