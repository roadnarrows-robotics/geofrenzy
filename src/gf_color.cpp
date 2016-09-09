/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%

//#include "usr/blink1/commaindline/blink1_light.h"
//#include "usb/blink1/commandline/blink1-lib.h"
#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"


#include "blink1/hiddata.h"
#include "blink1/blinkfn.h"


#include <signal.h>

#define IDENT_VENDOR_NUM        0x27B8
#define IDENT_PRODUCT_NUM       0x01ED
#define IDENT_VENDOR_STRING     "ThingM"
#define IDENT_PRODUCT_STRING    "blink(1)"
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



/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
void colorCallback(const std_msgs::ColorRGBA::ConstPtr& msg)
{
 // ROS_INFO("I heard: [%s]", msg->data.c_str());
 printf("rgba=\n");
 std::cout << msg->r;
 std::cout << msg->g;
 std::cout << msg->b;
 std::cout << msg->a;
  printf("convert\n");
  // for the sake of the conversion since there is no backgroup defines 
  // we are setting the background color to white (255,255,255)
 float a = (float)msg->a;
 float r = (((1.0-a) * 1.0) + (a * (msg->r/255))) * 255.0;
 float g = (((1.0-a) * 1.0 )+ (a * (msg->g/255))) * 255.0;
 float b = (((1.0-a) * 1.0 ) + (a * (msg->b/255))) * 255.0 ;
  std::cout << r;
  std::cout << "\n";
 std::cout << g;
   std::cout << "\n";
 std::cout << b;
 printf("done\n");
   std::cout << "\n";
 blink1_fadeToRGB(dev, 0.0, r, g, b);
 
}
// %EndTag(CALLBACK)%





//bool blinkCB(blink1::Blink::Request  &req,  blink1::Blink::Response &res)
//{
  //if (req.function == BL_FADE){
       //res.on=blink1_fadeToRGB(dev, req.t, req.r, req.g, req.b);
       //blink_control=false;
       //return true;
  //} 
  
  //if (req.function == BL_ON){       
       //res.on=blink1_fadeToRGB(dev, 0.0, req.r, req.g, req.b);
       //blink_control=false; 
       //return true;
  //}
  
  //if ((req.function == BL_BLINK) || (req.function == BL_RANDBLINK)){
       //blink_control=true;
       //last_request.function=req.function;
       //last_request.t=req.t;
       //last_request.r=req.r;
       //last_request.g=req.g;
       //last_request.b=req.b;
       //res.on=true;
       //return true;
  //}
//}




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

    switch(errCode){
        case USBOPEN_ERR_ACCESS:    return "Access to device denied";
        case USBOPEN_ERR_NOTFOUND:  return "The specified device was not found";
        case USBOPEN_ERR_IO:        return "Communication error with device";
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

    while((s = strtok(string, ", ")) != NULL && pos < buflen){
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
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "gf_color_led");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called colorCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("Blink", 1000, colorCallback);
// %EndTag(SUBSCRIBER)%

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%
