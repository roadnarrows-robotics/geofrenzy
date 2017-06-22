// GEOFRENZY FILE HEADER HERE

#ifndef _GF_ROS_H
#define _GF_ROS_H

#include <math.h>
#include <string>
#include "ros/ros.h"

namespace geofrenzy
{
  namespace gf_ros
  {
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Special values
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    const double  NoGeoPos    = -1000.0;  ///< no lat/long position

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Defaults
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    
    /*!
     * \brief Default minimum radius.
     *
     * Any new location that is less than this minimum is considered
     * insignificant by the fence server, and hence, no feature collections
     * are updated.
     *
     * Units: meters
     */
    const double  MinDistDft = 0.0;      ///< default min dist threshold (m)

    /*!
     * \brief Default region of interest radius.
     *
     * The Geofrenzy portal will only report features within the radius.
     *
     * Note:  ROS uses only SI units. The "natural" units for the Geofrenzy
     *        RoI are kilometers, so don't forget to convert.
     *
     * Units: meters
     */
    const double RoILevelDft = 6000.0;  ///< default region of interest (m)

    /*!
     * \brief Default map width for occupancy grid published by map server.
     *
     * Units: pixels
     */
    const int MapWidthDft = 250; ///< default occupancy grid width

    /*!
     * \brief Default map width for occupancy grid published by map server.
     *
     * Units: pixels
     */
    const int MapHeightDft = 250; ///< default occupancy grid height

    /*!
     * \brief Default map resolution for occupancy grid published by map server.
     *
     * Units: meters per pixel
     */
    const double MapResolutionDft = 0.2; ///< default occupancy grid resolution

    /*!
     * \brief Default point cloud horizontal resolution.
     *
     * Units: Number of steps. 
     */
    const int CloudWidthDft = 160;

    /*!
     * \brief Default point cloud vertical resolution.
     *
     * Units: Number of steps. 
     */
    const int CloudHeightDft = 120;

    /*!
     * \brief Default point cloud minimum horizontal field of view angle.
     *
     * Units: Radians
     */
    const double CloudHFoVMinDft = -M_PI_2;  ///< 90 degrees to the right

    /*!
     * \brief Default point cloud maximum horizontal field of view angle.
     *
     * Units: Radians
     */
    const double CloudHFoVMaxDft = M_PI_2;   ///< 90 degrees to the left

    /*!
     * \brief Default point cloud minimum vertical field of view angle.
     *
     * Units: Radians
     */
    const double CloudVFoVMinDft = M_PI_4;   ///< 45 degrees from up

    /*!
     * \brief Default point cloud maximum vertical field of view angle.
     *
     * Units: Radians
     */
    const double CloudVFoVMaxDft = M_PI_2; ///< 90 degrees from up (horizontal)

    /*!
     * \brief Default global frame for transformations
     *
     *
     */
    const std::string GlobalFrameDft = "map";   ///< default global frame

    /*!
     * \brief Default global frame for transformations
     *
     *
     */
    const std::string RobotFrameDft = "base_footprint";   ///< default global frame

    /*!
     * \brief Supported point cloud output formats.
     */
    enum CloudFmt
    {
      CloudFmtXYZ     = 0,  ///< FLOAT32 x-y-z format
      CloudFmtXYZRGB  = 1,  ///< FLOAT32 x-y-z, UINT8 r-g-b format
      CloudFmtXYZRGBA = 2  ///< FLOAT32 x-y-z, UINT8 r-g-b-a format
    };

    /*!
     * \brief Default published point cloud output format.
     *
     * Units: Enum
     */
    const int CloudPublishFmtDft = CloudFmtXYZRGB;


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // ROS Parameter Server interface
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Minimum distance parameter name.
     *
     * If the value is not set, MinDistDft is used.
     */
    const char *const ParamNameMinDist        = "geofrenzy_min_distance";

    /*!
     * \brief Region of interest parameter name.
     *
     * If the value is not set, RoILevelDft is used.
     */
    const char *const ParamNameRoILevel       = "geofrenzy_roi_level";

    /*!
     * \brief Default fence file parameter name.
     *
     * If the value is not set, then no file access is tried.
     */
    const char *const ParamNameFenceFilename  = "geofrenzy_fence_filename";

    /*!
     * \brief Cloud horizontal resolution parameter name.
     *
     * If the value is not set, then CloudWidthDft is used.
     */
    const char *const ParamNameCloudWidth = "geofrenzy_cloud_width";

    /*!
     * \brief Cloud vertical resolution parameter name.
     *
     * If the value is not set, then CloudHeightDft is used.
     */
    const char *const ParamNameCloudHeight = "geofrenzy_cloud_height";

    /*!
     * \brief Cloud horizontal FoV minimum angle parameter name.
     *
     * If the value is not set, then CloudHFoVMinDft is used.
     */
    const char *const ParamNameCloudHFoVMin = "geofrenzy_cloud_hfov_min";

    /*!
     * \brief Cloud horizontal FoV maximum angle parameter name.
     *
     * If the value is not set, then CloudHFoVMaxDft is used.
     */
    const char *const ParamNameCloudHFoVMax = "geofrenzy_cloud_hfov_max";

    /*!
     * \brief Cloud vertical FoV minimum angle parameter name.
     *
     * If the value is not set, then CloudVFoVMinDft is used.
     */
    const char *const ParamNameCloudVFoVMin = "geofrenzy_cloud_vfov_min";

    /*!
     * \brief Cloud vertical FoV maximum angle parameter name.
     *
     * If the value is not set, then CloudVFoVMaxDft is used.
     */
    const char *const ParamNameCloudVFoVMax = "geofrenzy_cloud_vfov_max";

    /*!
     * \brief Cloud published output format parameter name.
     *
     * If the value is not set, then CloudPublishFmtDft is used.
     */
    const char *const ParamNameCloudPublishFmt = "geofrenzy_cloud_out_fmt";

    /*!
     * \brief Map width parameter name
     *
     * If the value is not set, then MapWidthDft is used.
     */
    const char *const ParamNameMapWidth = "geofrenzy_map_width";

    /*!
     * \brief Map height parameter name
     *
     * If the value is not set, then MapHeightDft is used.
     */
    const char *const ParamNameMapHeight = "geofrenzy_map_height";

    /*!
     * \brief Map resoltion parameter name
     *
     * If the value is not set, then MapResolutionDft is used.
     */
    const char *const ParamNameMapResolution = "geofrenzy_map_resolution";

    /*!
     * \brief Global Frame parameter name.
     *
     * If the value is not set, then GlobalFrameDft is used.
     */
    const char *const ParamNameGlobalFrame = "geofrenzy_global_frame";

    /*!
     * \brief Robot Frame parameter name.
     *
     * If the value is not set, then RobotFrameDft is used.
     */
    const char *const ParamNameRobotFrame = "geofrenzy_robot_frame";

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Geofrenzy ROS node root names
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief The Geofrenzy ROS portal fence server root name.
     *
     * Since all running ROS nodes must have unique names, the actual server
     * name is constructed as follows:
     * ~~~
     * <root>_<gf_class_idx>
     * ~~~
     */
    const char *const NodeRootFenceServer   = "gf_server";

    /*!
     * \brief The Geofrenzy ROS map server root name.
     *
     * Since all running ROS nodes must have unique names, the actual server
     * name is constructed as follows:
     * ~~~
     * <root>_<gf_class_idx>
     * ~~~
     */
    const char *const NodeRootMapServer  = "gf_map_server";
  
    /*!
     * \brief The Geofrenzy ROS cloud virtual sensor node.
     *
     * Since all running ROS nodes must have unique names, the actual server
     * name is constructed as follows:
     * ~~~
     * <root>_<gf_class_idx>
     * ~~~
     */
    const char *const NodeRootVCloud   = "gf_vcloud";

  
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // ROS subscribed and published (sub)topic names
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    const char *const TopicNameFix    = "/fix";
    const char *const TopicNameFcJson = "geofrenzy/featureCollection/json";
    const char *const TopicNameFcGeo  = "geofrenzy/featureCollection/geo";
    const char *const TopicNameFcDist = "geofrenzy/featureCollection/distance";
    const char *const TopicNameMap    = "geofrenzy/map";
    const char *const TopicNameMapMD  = "geofrenzy/map_metadata";
    const char *const TopicNameCloud  = "geofrenzy/cloud";
  
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // ROS services
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    const char *const ServiceNameGetEntitlement = "geofrenzy/get_entitlement";
    const char *const ServiceNameGetEntitlementList =
                                              "geofrenzy/get_entitlement_list";
    const char *const ServiceNameGetMap = "geofrenzy/get_map";
  
    //
    // Types
    //

    /*! map of ROS server services type */
    typedef std::map<std::string, ros::ServiceServer> MapServices;

    /*! map of ROS client services type */
    typedef std::map<std::string, ros::ServiceClient> MapClientServices;

    /*! map of ROS publishers type */
    typedef std::map<std::string, ros::Publisher> MapPublishers;

    /*! map of ROS subscriptions type */
    typedef std::map<std::string, ros::Subscriber> MapSubscriptions;

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // ROS utilities
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Parse command line argument to determine Geofrenzy class index.
     *
     * \note
     * This function should be call prior to ros::init() to make node name.
     *
     * Format: _gf_class_idx:=<class_idx>
     *
     * \param argc  Command line argument count.
     * \param argv  Command line arguments.
     * \param dft   Geofrenzy class index default (if no value is specified).
     *
     * \return Geofrenzy class index.
     */
    uint64_t paramClassIndex(int argc, char *argv[], uint64_t dft=1);
 
    /*!
     * \brief Make an unique Geofrenzy node from the class index.
     *
     * \param strRoot       Node root prefix string.
     * \param gf_class_idx  Geofrenzy class index.
     *
     * \return String <root>_<idx>
     */
   std::string makeNodeName(const std::string strRoot, uint64_t gf_class_idx);
    
    /*!
     * \brief Make a gf_server dwell topic name.
     *
     * Output:
     * ```
     * <node_name>/geofrenzy/<ent_idx>/dwell/<ent_base>
     * ```
     * \param gf_class_idx    Geofrenzy class index.
     * \param gf_ent_idx      Geofrenzy entitlement index.
     * \param gf_ent_subtype  Geofrenzy entitlement subtype (base or json).
     *
     * \return String.
     */
    std::string makeDwellTopicName(uint64_t          gf_class_idx,
                                   uint64_t          gf_ent_idx,
                                   const std::string gf_ent_subtype);

    /*!
     * \brief Fill in ROS standard message header.
     *
     * \param [in,out] header Message header.
     * \param nSeqNum         Sequence number.
     * \param strFrameId      Frame id. No frame = "0", global frame = "1".
     */
    void stampHeader(std_msgs::Header  &header,
                     const int32_t     nSeqNum = 0,
                     const std::string strFrameId = "geofrenzy");

    /*!
     * \brief Fill in ROS standard message header.
     *
     * \param [in,out] header Message header.
     * \param stamp           Time stamp.
     * \param nSeqNum         Sequence number.
     * \param strFrameId      Frame id. No frame = "0", global frame = "1".
     */
    void stampHeader(std_msgs::Header  &header,
                     const ros::Time   &stamp,
                     const int32_t     nSeqNum = 0,
                     const std::string strFrameId = "geofrenzy");

 
  } // namespace gf_ros
} // namespace geofrenzy

#endif // _GF_ROS_H
