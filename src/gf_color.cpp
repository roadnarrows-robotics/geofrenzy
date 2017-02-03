
#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"

#include "blink1/hiddata.h"
#include "blink1/blinkfn.h"

#include <signal.h>

#define IDENT_VENDOR_NUM        0x27B8
#define IDENT_PRODUCT_NUM       0x01ED
#define IDENT_VENDOR_STRING     "ThingM"
#define IDENT_PRODUCT_STRING    "blink(1)"

/**
 * This node subscribes to a topic "Blink" and uses the Ros ColorRGBA messages to set
 * the color of the Blink usb led display https://thingm.com/products/blink-1/
 */


//add_executable(blink1_example examples/blink1_example.cpp)
//target_link_libraries(blink1_example ${catkin_LIBRARIES})
//add_dependencies(blink1_example blink1_gencpp)

usbDevice_t *dev;
bool blink_control;
bool exit_;

int blink1_open(usbDevice_t **dev);
char *blink1_error_msg(int errCode);
void blink1_close(usbDevice_t *dev);
int blink1_fadeToRGB(usbDevice_t *dev, int fadeMillis,
                     uint8_t r, uint8_t g, uint8_t b );
int blink1_setRGB(usbDevice_t *dev, uint8_t r, uint8_t g, uint8_t b );
static int  hexread(char *buffer, char *string, int buflen);

void SigintHandler(int sig)
{
    exit_=true;
    blink1_fadeToRGB(dev, 1000, 0, 0, 0);
    ros::Duration(1).sleep();
    blink1_close(dev);
    ros::Duration(0.5).sleep();
    ros::shutdown();
}

void colorCallback(const std_msgs::ColorRGBA::ConstPtr& msg)
{
    ROS_DEBUG("IN Red=%f,Blue=%f,Green=%f,Alpha=%f",msg->r,msg->b,msg->g,msg->a);
    // for the sake of the conversion since there is no backgroup defines
    // we are setting the background color to white (255,255,255)
    float a = (float)msg->a;
    float r = (((1.0-a) * 1.0) + (a * (msg->r/255))) * 255.0;
    float g = (((1.0-a) * 1.0 )+ (a * (msg->g/255))) * 255.0;
    float b = (((1.0-a) * 1.0 ) + (a * (msg->b/255))) * 255.0 ;
    ROS_DEBUG("Out Red=%f,Blue=%f,Green=%f",r,b,g);
    blink1_fadeToRGB(dev, 0.0, r, g, b);

}


/**
 * Open up a blink(1) for transactions.
 * returns 0 on success, and opened device in "dev"
 * or returns non-zero error that can be decoded with blink1_error_msg()
 * FIXME: what happens when multiple are plugged in?
 */
int blink1_open(usbDevice_t **dev)
{
    return usbhidOpenDevice(dev,
                            IDENT_VENDOR_NUM,  NULL,
                            IDENT_PRODUCT_NUM, NULL,
                            1);  // NOTE: '0' means "not using report IDs"
}

/**
 * Close a Blink1
 */
void blink1_close(usbDevice_t *dev)
{
    usbhidCloseDevice(dev);
}

/**
 *
 */
int blink1_fadeToRGB(usbDevice_t *dev, int fadeMillis,
                     uint8_t r, uint8_t g, uint8_t b )
{
    char buf[9];
    int err;

    if( dev==NULL ) {
        return -1; // BLINK1_ERR_NOTOPEN;
    }

    int dms = fadeMillis/10;  // millis_divided_by_10

    buf[0] = 1;
    buf[1] = 'c';
    buf[2] = r;
    buf[3] = g;
    buf[4] = b;
    buf[5] = (dms >> 8);
    buf[6] = dms % 0xff;
    buf[7] = 0; // ledn
    //buf[8] = 0; // unused

    if( (err = usbhidSetReport(dev, buf, sizeof(buf))) != 0) {
        ROS_ERROR("fadeToRGB: error writing: %s\n",blink1_error_msg(err));
    }
    return err;
}

/**
 *
 */
int blink1_setRGB(usbDevice_t *dev, uint8_t r, uint8_t g, uint8_t b )
{
    char buf[9];
    int err;

    if( dev==NULL ) {
        return -1; // BLINK1_ERR_NOTOPEN;
    }

    buf[0] = 1;
    buf[1] = 'n';
    buf[2] = r;
    buf[3] = g;
    buf[4] = b;

    if( (err = usbhidSetReport(dev, buf, sizeof(buf))) != 0) {
        ROS_ERROR("setRGB: error writing: %s\n",blink1_error_msg(err));
    }
    return err;
}


//
char *blink1_error_msg(int errCode)
{
    static char buffer[80];

    switch(errCode) {
    case USBOPEN_ERR_ACCESS:
        sprintf(buffer,"Access to device denied");
        return buffer;
    case USBOPEN_ERR_NOTFOUND:
        sprintf(buffer,"The specified device was not found");
        return buffer;
    case USBOPEN_ERR_IO:
        sprintf(buffer,"Communication error with device");
        return buffer;
    default:
        sprintf(buffer, "Unknown USB error %d", errCode);
        return buffer;
    }
    return NULL;    /* not reached */
}

//
static int  hexread(char *buffer, char *string, int buflen)
{
    char    *s;
    int     pos = 0;

    while((s = strtok(string, ", ")) != NULL && pos < buflen) {
        string = NULL;
        buffer[pos++] = (char)strtol(s, NULL, 0);
    }
    return pos;
}


int main(int argc, char **argv)
{
    signal(SIGINT, SigintHandler);

    if( blink1_open(&dev) ) {
        ROS_ERROR("error: couldn't open blink1\n");
        exit(1);
    }

    ros::init(argc, argv, "gf_color_led");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("Blink", 2, colorCallback);

    ros::spin();

    return 0;
}
