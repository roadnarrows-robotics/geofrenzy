////////////////////////////////////////////////////////////////////////////////
//
// Package:   Geofrenzy Robot ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/geofrenzy
//
// File:      gf_breach_inhibitor.cpp
//
/*! \file
 *
 * \brief The Geofrenzy ROS fence breach inhibitor node source.
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
#include <math.h>
#include <string>
#include <iostream>
#include <ctime>

//
// ROS
//
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

//
// ROS generated Geofrenzy messages
//
#include "geofrenzy/GfDistFeatureCollection.h"
#include "geofrenzy/GfDistFeature.h"
#include "geofrenzy/GfDwellThreshold.h"

//
// mavros
//
#include "mavros_msgs/ParamSet.h"
#include "mavros_msgs/HomePosition.h"

//
// Geofrenzy
//
#include "gf_ros.h"
#include "gf_types.h"
#include "gf_sentinel.h"

using namespace geofrenzy::gf_ros;

namespace geofrenzy
{
    class BreachInhibitor
    {
        /*!
        * This class is responisible for inhibiting fence breaches
        * by setting firmware parameters intended to slow down a
        * drone that is approaching a fence perimeter.
        * Work in progress.
        */
        public:
            void featureCollectionCallback(const geofrenzy::GfDistFeatureCollection distFeatures);
            void dwellCallbackThreshold(const GfDwellThreshold &dwellMsg);
            void cbHomePos(const mavros_msgs::HomePosition &msgHomePos);
            void cbGlobalPos(const sensor_msgs::NavSatFix &msgFix);

            //void advertisePublishers(int nQueueDepth=1);
            void subscribeToTopics(int nQueueDepth=5);
            //void advertiseServices();
            void clientServices(ros::NodeHandle &nh);
            void initBreachInhibitorProperties();

            BreachInhibitor(ros::NodeHandle &nh) :  m_nh(nh){}

        private:
            ros::NodeHandle &m_nh;      ///< node handle
            uint64_t        m_fenceClass;    ///< geofrenzy class index

            //ROS publishers/subscribers
            gf_ros::MapPublishers     m_publishers;     ///< topic publishers
            gf_ros::MapSubscriptions  m_subscriptions;  ///< topic subscriptions
            gf_ros::MapClientServices m_clientServices; ///< client services

            GfClassIndex  m_gciServer;           ///< geofrenzy portal server class index
            GfEntBaseThreshold  m_threshEnt;     ///< flight thresholds
            GfEntitlementIndex m_geiAltitude;    ///< altitude entitlement index

            std::string m_serviceParamSet;       ///< parameter set
            std::string m_paramSetMaxTilt;       ///< Max tilt
            std::string m_paramSetMaxAscentVel;  ///< Maximum ascent velocity
            std::string m_paramSetMaxDescentVel; ///< Maximum ascent velocity
            std::string m_fcTopicName;           ///< Feature Collection Dist
            std::string m_altTopicName;          ///< Altitude Threshold topic
            std::string m_topicHomePos;          ///< home position topic
            std::string m_topicGlobalPos;        ///< global position topic


            GfSentinelMav::GeoPos  m_posCur;     ///< current geographic position
            GfSentinelMav::GeoPos  m_posHome;    ///< current home position

            bool m_defaultMaxTiltSet;            ///< default tilt angle set
            bool m_defaultMaxAVSet;              ///< default ascent velocity set
            bool m_hasHomePos;
            bool m_hasCurrentPos;

            double getDistanceToLine(geometry_msgs::Point p1, geometry_msgs::Point p2);
            void updateMaxTilt(double distance);
            void updateMaxAscentVel();
            bool reqSetParam(std::string paramId, double value);
	    clock_t m_lastParamSetTime;
    };

    /*!
     * \brief Init properties. Check for topic names on param server.
     *
     */
    void BreachInhibitor::initBreachInhibitorProperties(){
        int32_t     val;
        int32_t     dft = (int32_t)GciGuestAcct;
        //
        // Get the gefrenzy portal server class index.
        //
        // Note: no 64-bit types in parameter server
        //
        dft = (int32_t)GciGuestAcct;
        m_nh.param(ParamNameSrServerGci, val, dft);
        m_gciServer = (GfClassIndex)val;

        //TODO: Check if get param matches default
        m_defaultMaxTiltSet = true;

        //Set FW param service
        m_serviceParamSet  = "/mavros/param/set";
        m_paramSetMaxTilt  = "MPC_TILTMAX_AIR";
        m_paramSetMaxAscentVel = "MPC_Z_VEL_MAX_UP";
        m_paramSetMaxDescentVel = "MPC_Z_VEL_MAX_DN";

        //
        // Use Feature Collection Distance and always treat as No-Exit
        // TODO: Need to check for No-Exit vs No-Entry on class
        std::stringstream ss;
        ss << "/" << makeNodeName(NodeRootFenceServer, m_gciServer) << "/" << TopicNameFcDist;
        m_fcTopicName = ss.str();

        //
        // Watch for an altitude threshold topic entitlement
        //
        dft = (int32_t)GeiFlightAltitudes;
        m_nh.param(ParamNameSrAltitudeGei, val, dft);
        m_geiAltitude = (GfEntitlementIndex)val;
        std::string slash = "/";
        m_altTopicName = slash + makeDwellTopicName(m_gciServer, m_geiAltitude, GfEntDataTypeThreshold);

        m_topicGlobalPos  = "/mavros/global_position/global";
        m_topicHomePos    = "/mavros/home_position/home";
	m_lastParamSetTime = clock();

    }

    double BreachInhibitor::getDistanceToLine(geometry_msgs::Point p1, geometry_msgs::Point p2){
        double x = 0.0;
        double y = 0.0;

        double a = x - p1.x;
        double b = y - p1.y;
        double c = p2.x - p1.x;
        double d = p2.y - p1.y;

        double dotProd = a*c + b*d;
        double lenSq   = c*c + d*d;

        double param = -1;

        //Check for 0 length line
        if (lenSq != 0){
            param = dotProd/lenSq;
        }

        double xx, yy;

        if (param < 0){
            xx = p1.x;
            yy = p1.y;
        }else if(param > 1){
            xx = p2.x;
            yy = p2.y;
        }else{
            xx = p1.x + param*c;
            yy = p1.y + param*d;
        }

        double dx = x - xx;
        double dy = y - yy;
        return sqrt(dx*dx + dy*dy);
    }

    void BreachInhibitor::updateMaxTilt(double distance){
        ROS_INFO_STREAM("Min Distance: " << distance << "meters");
        double defaultAngle = 45.0;
        double threshHold = 10.0;
        double cushion = 0.5;
        double minAngle = 5.0;
        if(distance < threshHold){
         double maxAngle = (defaultAngle)*(cushion - ((threshHold-distance)/threshHold));
         if(maxAngle < minAngle){
             maxAngle = minAngle;
         }
         ROS_INFO_STREAM("Max Tilt: " << maxAngle << "degrees");
         reqSetParam(m_paramSetMaxTilt, maxAngle);
         m_defaultMaxTiltSet = false;
        }else{
            if(!m_defaultMaxTiltSet){
                if(reqSetParam(m_paramSetMaxTilt, defaultAngle)){
                    m_defaultMaxTiltSet = true;
                }
            }
        }
    }

    void BreachInhibitor::updateMaxAscentVel(){
        double defaultVelocity = 3.0;
        double threshHold = 5.0;
        double cushion = 0.8;
        double minVelocity = 0.5;

        if(m_hasHomePos && m_hasCurrentPos){
            double flightCeiling = m_posHome.m_altitude + m_threshEnt.m_upper;
            double distance = flightCeiling - m_posCur.m_altitude;

            if(distance < threshHold){
             double maxVelocity = (defaultVelocity)*(cushion - ((threshHold-distance)/threshHold));
             if(maxVelocity < minVelocity){
                 maxVelocity = minVelocity;
             }
             ROS_INFO_STREAM("Max Vertical Ascent Velocity: " << maxVelocity << "m/s");
             reqSetParam(m_paramSetMaxAscentVel, maxVelocity);
             m_defaultMaxAVSet = false;
            }else{
                if(!m_defaultMaxAVSet){
                    if(reqSetParam(m_paramSetMaxAscentVel, defaultVelocity)){
                        m_defaultMaxAVSet = true;
                    }
                }
            }
        }else{
            if(!m_defaultMaxAVSet){
                if(reqSetParam(m_paramSetMaxAscentVel, defaultVelocity)){
                    m_defaultMaxAVSet = true;
                }
            }
        }
    }

    bool BreachInhibitor::reqSetParam(std::string paramId, double value){
        
	clock_t now = clock();
	double timeSinceLast = (now - m_lastParamSetTime)/(double)CLOCKS_PER_SEC;
	//if(timeSinceLast <= 0.01){
	//  return false;
	//}else{
	//  m_lastParamSetTime = now;
	//}
	
	std::string     &nameSvc = m_serviceParamSet;
        mavros_msgs::ParamSet  svc;

        svc.request.param_id = paramId;
        svc.request.value.integer = 0;
        svc.request.value.real = (int)value;

        if(m_clientServices[nameSvc].call(svc) )
        {
          ROS_DEBUG_STREAM("Breach Inhibitor" << ": " << nameSvc);
          return svc.response.success;
        }
        else
        {
          ROS_ERROR_STREAM("Breach Inhibitor" << ": " << "Service " << nameSvc << " failed.");
          return false;
        }
    }

    /**
     * FeatureCollection callback. This callback calculates minimum distance
     * to the fence based on the feature collection. It also triggers setting
     * maximum tilt and thrust variables.
     */
    void BreachInhibitor::featureCollectionCallback(const geofrenzy::GfDistFeatureCollection distFeatures){

        double minDistance = 1000.0; //init to large min distance
        geofrenzy::GfDistFeatureCollection featureCollection = distFeatures;
        for(std::vector<geofrenzy::GfDistFeature>::iterator feature_it = featureCollection.features.begin();
            feature_it != featureCollection.features.end(); feature_it++){
            geofrenzy::GfDistFeature feature = *(feature_it);
            for(std::vector<geofrenzy::Polygon64>::iterator geometry_it = feature.geometry.begin();
                geometry_it != feature.geometry.end(); ++geometry_it){
                geofrenzy::Polygon64 geometry = *(geometry_it);
                for(std::vector<geometry_msgs::Point>::iterator point_it = geometry.points.begin();
                    point_it != geometry.points.end(); ++point_it){
                    geometry_msgs::Point p1 = *point_it;
                    geometry_msgs::Point p2;
                    if((point_it+1) != geometry.points.end()){
                        p2 = *(point_it+1);
                    }else{
                        p2 = *(geometry.points.begin());
                    }
                    double d = getDistanceToLine(p1, p2);
                    if (d < minDistance){
                        minDistance = d;
                    }
                }
            }
        }
        updateMaxTilt(minDistance);
        updateMaxAscentVel();
    }

    void BreachInhibitor::dwellCallbackThreshold(const GfDwellThreshold &dwellMsg)
    {
        //ROS_INFO_STREAM("dwellCallbackThreshold = " << dwellMsg);

        m_threshEnt.m_lower = dwellMsg.entitlement.threshold_lower;
        m_threshEnt.m_upper = dwellMsg.entitlement.threshold_upper;
        m_threshEnt.m_units = dwellMsg.entitlement.threshold_unit;
    }

    void BreachInhibitor::cbGlobalPos(const sensor_msgs::NavSatFix &msgFix)
    {
        m_posCur.m_latitude  = msgFix.latitude;
        m_posCur.m_longitude = msgFix.longitude;
        m_posCur.m_altitude  = msgFix.altitude;
        m_hasCurrentPos      = true;
    }

    void BreachInhibitor::cbHomePos(const mavros_msgs::HomePosition &msgHomePos)
    {
        //
        // Set home (RTL) position.
        //
        m_posHome.m_latitude  = msgHomePos.latitude;
        m_posHome.m_longitude = msgHomePos.longitude;
        m_posHome.m_altitude  = msgHomePos.altitude;
        m_hasHomePos          = true;
    }


    /*!
     * \brief Subscribe to all topics.
     *
     * \param nQueueDepth   Maximum queue depth.
     */
    void BreachInhibitor::subscribeToTopics(int nQueueDepth)
    {
        m_subscriptions[m_fcTopicName] =
                m_nh.subscribe(m_fcTopicName,
                               nQueueDepth, &BreachInhibitor::featureCollectionCallback, &(*this));
        m_subscriptions[m_altTopicName] =
                m_nh.subscribe(m_altTopicName,
                               nQueueDepth, &BreachInhibitor::dwellCallbackThreshold, &(*this));
        m_subscriptions[m_topicGlobalPos] =
          m_nh.subscribe(m_topicGlobalPos, 1, &BreachInhibitor::cbGlobalPos, &(*this));
        m_subscriptions[m_topicHomePos] =
          m_nh.subscribe(m_topicHomePos, 1, &BreachInhibitor::cbHomePos, &(*this));
    }

    void BreachInhibitor::clientServices(ros::NodeHandle &nh)
    {
        // request set parameter
        m_clientServices[m_serviceParamSet] =
          nh.serviceClient<mavros_msgs::ParamSet>(m_serviceParamSet);
    }

}

/**
 * This node subscribes to distance feature collection topic
 * and limits speeds based on the distance from the fence
 *
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, NodeRootBreachInhibitor);

    ros::NodeHandle nh(NodeRootBreachInhibitor);

    geofrenzy::BreachInhibitor bi(nh);
    bi.initBreachInhibitorProperties();
    //bi.advertisePublishers();
    bi.subscribeToTopics();
    //bi.advertiseServices();
    bi.clientServices(nh);

    //std::cerr << bi << std::endl;

    while(ros::ok()){
        ros::spin();
    }
}
