#include "bag_flip_detection_node.h"

BagFlipDetection::BagFlipDetection(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of ExampleRosClass");
    initializeSubscribers();
    initializePublishers();
}

//member helper function to set up publishers;
void BagFlipDetection::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    image_transport::ImageTransport it(nh_);
    pub = it.advertise("camera/image", 1);
}


void BagFlipDetection::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    sub = nh_.subscribe("/flip_camera/image_bag_raw", 1000, &BagFlipDetection::imageCallback, this);
}


void BagFlipDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  cv::Mat image_raw = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8) -> image; 
  cv::Mat image_hsv, mask, result;
  cvtColor(image_raw, image_hsv, COLOR_BGR2HSV);
  inRange(image_hsv, Scalar(5, 50, 50), Scalar(15, 255, 255), mask);

  findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  Mat drawing = Mat::zeros( mask.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( 255, 255, 255 );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
     }

  //std::cout << image_hsv.channels() << endl;
  std::cout << contours.size() << endl;
  //imshow("nut", result);
  //waitKey(30);
  sensor_msgs::ImagePtr imsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawing).toImageMsg();
  pub.publish(imsg);
}

int main(int argc, char **argv)
{
  //initialize node
  ros::init(argc, argv, "bagFlipDetection");

  // node handler
  ros::NodeHandle nh;

  BagFlipDetection bagFlipDetection(&nh);

  ros::spin();

  return 0;
}
