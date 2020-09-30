#ifndef BAG_FLIP_DETECTION_NODE_H_
#define BAG_FLIP_DETECTION_NODE_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this
#include <iostream>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>

using namespace std;
using namespace cv;

// define a class, including a constructor, member variables and member functions
class BagFlipDetection
{
public:
    BagFlipDetection(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    // some objects to support subscriber and publisher
    ros::Subscriber sub;
    image_transport::Publisher pub;
    
    // member methods as well:
    void initializeSubscribers();
    void initializePublishers();
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};
#endif
