#include <algorithm>
#include <string>

#include <ros/ros.h>
#include "geofrenzy/entitlement.h"
#include "std_msgs/ColorRGBA.h"

/**
 * This node subscribes to a topic of "dwell" with a message type of gf_entitlement and
 * publishes a topic of "Blink" with a Ros ColorRGBA message set according to the entitlement string
 * @todo modify program to recognize the the specific color entitlement when it is available in the string
 */

class EntitlementToBlink
{
  public:
    EntitlementToBlink(std::string class_list)
    {
		
		std::string topic_prefix =  "geofrenzy/";
		std::string topic_postfix = "/dwell/json";
		std::string color_ent_string = "0";
        pub_ = n_.advertise<std_msgs::ColorRGBA>("/Blink", 1);
        sub_ = n_.subscribe("/dwell", 1, &EntitlementToBlink::callback, this);
    }

    void callback(const geofrenzy::entitlement &input)
    {
        //00 eb ff ff ffffffffffffffffffff
        std::string dwell = input.dwell;
        std::string entitlement = input.entitlement;
        std_msgs::ColorRGBA msg;
        if (dwell.compare("i") == 0)
        {
            std::string redstring = entitlement.substr(0, 2);
            std::transform(redstring.begin(), redstring.end(), redstring.begin(), ::toupper);
            std::string bluestring = entitlement.substr(2, 2);
            std::transform(bluestring.begin(), bluestring.end(), bluestring.begin(), ::toupper);
            std::string greenstring = entitlement.substr(4, 2);
            std::transform(greenstring.begin(), greenstring.end(), greenstring.begin(), ::toupper);
            std::string alphastring = entitlement.substr(6, 2);
            std::transform(alphastring.begin(), alphastring.end(), alphastring.begin(), ::toupper);
            ROS_DEBUG("red=%s,blue=%s,green=%s,alpha=%s", redstring.c_str(), bluestring.c_str(), greenstring.c_str(), alphastring.c_str());
            std::stringstream ssr, ssb, ssg, ssa;
            ssr << std::hex << redstring;
            unsigned int redInt;
            ssr >> redInt;
            ssg << std::hex << greenstring;
            unsigned int greenInt;
            ssg >> greenInt;
            ssb << std::hex << bluestring;
            unsigned int blueInt;
            ssb >> blueInt;
            ssa << std::hex << alphastring;
            unsigned int alphaInt;
            ssa >> alphaInt;

            ROS_DEBUG("red=%d,blue=%d,green=%d,alpha=%d", redInt, blueInt, greenInt, alphaInt);

            msg.r = (float)redInt;
            msg.g = (float)greenInt;
            msg.b = (float)blueInt;
            msg.a = (float)((float)alphaInt / 255.0);
            pub_.publish(msg);
        }
        else
        {
            msg.g = 255;
            msg.b = 255;
            msg.a = 1.0;
            msg.r = 255;
        }
        pub_.publish(msg);
    }

  private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

}; //End of class EntitlementToBlink

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "Entitlement_to_Blink");

    //Create an object of class EntitlementToBlink that will take care of everything
    EntitlementToBlink SAPObject;

    ros::spin();

    return 0;
}
