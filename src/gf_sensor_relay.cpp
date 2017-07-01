// GEOFRENZY FILE HEADER HERE


//
// ROS
//
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

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
#include "gf_ros.h"

/*!
 * \brief Entitlement data type enumeration.
 */
enum EntDataType
{
  EntDataTypeUndef,     // undefined or unknown
  EntDataTypeBoolset,   // boolean bit set
  EntDataTypeColor,     // red-green-blue-alpha
  EntDataTypeProfile,   // profile number
  EntDataTypeThreshold  // threshold fpn triple
};

/*!
 * \brief Message type enumeration.
 */
enum MsgType
{
  MsgTypeUndef,         // undefined or unknown
  MsgTypeImage          // boolean bit set
};

class SensorRelay
{
    /*!
    * This class is responsible for registering incoming/outgoing
    * ros topics that are controlled by geofrenzy entitlements.
    */
    public:
        void sensorCallbackImage(const sensor_msgs::Image img);
        void dwellCallbackBoolset(const geofrenzy::GfDwellBoolset dwell);
        void advertisePublishers(int nQueueDepth=1);
        void subscribeToTopics(int nQueueDepth=5);
        void initSensorRelayProperties();

        SensorRelay(ros::NodeHandle &nh) : m_nh(nh){}

    private:

        ros::NodeHandle &m_nh;      ///< node handle

        bool m_sensorPermitted;     ///< Sensor permission value

        //Messaging parameters
        std::string m_topicIn;      ///< Topic sensor publishes to
        std::string m_topicOut;     ///< Topic consumers subscribe to
        std::string m_dwellTopic;   ///< Topic for dwell message
        EntDataType m_entDataType;  ///< Data type for dwell message
        MsgType     m_msgType;      ///< Message type for relay topic

        //ROS publishers/subscribers
        geofrenzy::gf_ros::MapPublishers     m_publishers;     ///< Geofrenzy map server publishers
        geofrenzy::gf_ros::MapSubscriptions  m_subscriptions;  ///< Geofrenzy map server subscriptions

        //Image to publish when camera not permitted
        sensor_msgs::Image m_defaultImage;


};

/*!
 * \brief Init properties. Check for topic names on param server.
 *
 */
void SensorRelay::initSensorRelayProperties(){
    /*!
     * This should check param server and args for values
     */

    //Topic in message type should be resolved as well
    m_topicIn = "/laelaps/laelaps_camera/image_raw";
    m_msgType = MsgTypeImage;

    //Topic out should come from param server, topic type same as topicIn
    m_topicOut = "/geofrenzy/image_raw";

    //Dwell topic type needs to be resolved as well
    m_dwellTopic = "/gf_server_168/gf_server_168/geofrenzy/209/dwell/boolset";
    m_entDataType = EntDataTypeBoolset;

    //Turn local image into ros message to publish
    std_msgs::Header header;
    cv::Mat img = cv::imread("/home/woundzoom/catkin_ws/src/geofrenzy/images/default.png", CV_LOAD_IMAGE_COLOR);
    cv_bridge::CvImage img_bridge;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
    img_bridge.toImageMsg(m_defaultImage);
    //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
}

/*!
 * \brief Subscribe to all topics.
 *
 * \param nQueueDepth   Maximum queue depth.
 */
void SensorRelay::subscribeToTopics(int nQueueDepth)
{
    //Use topic message type to assign the appropriate sensor callback
    switch( m_msgType )
    {
        case MsgTypeImage:
            m_subscriptions[m_topicIn] = m_nh.subscribe(m_topicIn, nQueueDepth,
                                                        &SensorRelay::sensorCallbackImage, &(*this));
            break;
        case MsgTypeUndef:
            ROS_INFO("Unknown message type. Not subscribing to topic");
            break;
        default:
            break;
    }

    //Use entitlement data type to assign appropriate dwell callback
    switch( m_entDataType )
    {
        case EntDataTypeBoolset:
            m_subscriptions[m_dwellTopic] = m_nh.subscribe(m_dwellTopic, nQueueDepth,
                                                        &SensorRelay::dwellCallbackBoolset, &(*this));
            break;
        case MsgTypeUndef:
            ROS_INFO("Unknown message type. Not subscribing to topic");
            break;
        default:
            break;
    }

}

/*!
 * \brief Advertise all publishers.
 *
 * \param nQueueDepth   Maximum queue depth.
 */
void SensorRelay::advertisePublishers(int nQueueDepth)
{
    //Use topic message type to advertise appropriate message
    switch( m_msgType )
    {
        case MsgTypeImage:
            m_publishers[m_topicOut] = m_nh.advertise<sensor_msgs::Image>(m_topicOut, nQueueDepth, true);
            break;
        case MsgTypeUndef:
            ROS_INFO("Unknown message type. Not subscribing to topic");
            break;
        default:
            break;
    }
}

void SensorRelay::sensorCallbackImage(const sensor_msgs::Image img)
{
    if(m_sensorPermitted){
        m_publishers[m_topicOut].publish(img);
    }else{
        m_publishers[m_topicOut].publish(m_defaultImage);
    }
}

void SensorRelay::dwellCallbackBoolset(const geofrenzy::GfDwellBoolset dwellMsg){
    if((dwellMsg.entitlement.bitset_num == 0) && (dwellMsg.entitlement.ent_header.dwell)){
        m_sensorPermitted = false;
    }else{
        m_sensorPermitted = true;
    }

}


/**
 * This node subscribes to dwell topics and determines permissions for
 * various robot sensors. The node subscribes to relevant sensor
 * topics and either relays them or not based on permissions. This is
 * not likely to be the permanent solution, as gf_server will resolve
 * permissions and publish topics targeted at specific subsystems.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gf_sensor_relay");

    ros::NodeHandle nh;

    SensorRelay sr(nh);
    sr.initSensorRelayProperties();
    sr.advertisePublishers();
    sr.subscribeToTopics();

    while(ros::ok()){
        ros::spin();
    }
}
