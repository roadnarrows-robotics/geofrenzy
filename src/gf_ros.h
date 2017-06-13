// GEOFRENZY FILE HEADER HERE

#ifndef _GF_ROS_H
#define _GF_ROS_H

#include <string>

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
     * \brief Default fence file name.
     *
     * If the value is not set, then no file access is tried.
     */
    const char *const ParamNameFenceFilename  = "geofrenzy_fence_filename";


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
    const char *const NodeRootMapperServer  = "gf_map_server2";
  
  
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // ROS subscribed and published (sub)topic names
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    const char *const TopicNameFix    = "/fix";
    const char *const TopicNameFcJson = "geofrenzy/featureCollection/json";
    const char *const TopicNameFcGeo  = "geofrenzy/featureCollection/geo";
    const char *const TopicNameFcDist = "geofrenzy/featureCollection/distance";
  
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // ROS services
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    const char *const ServiceNameGetEntitlement = "geofrenzy/get_entitlement";
    const char *const ServiceNameGetEntitlementList =
                                              "geofrenzy/get_entitlement_list";
  

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

 
  } // namespace gf_ros
} // namespace geofrenzy

#endif // _GF_ROS_H
