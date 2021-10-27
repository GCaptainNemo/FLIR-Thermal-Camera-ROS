#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv/highgui.h>


#define ROS_MIN_MAJOR 1
#define ROS_MIN_MINOR 8
#define ROS_MIN_PATCH 16

#if ROS_VERSION_MINIMUM(ROS_MIN_MAJOR, ROS_MIN_MINOR, ROS_MIN_PATCH)
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/CvBridge.h>
#endif

#include <utils.h>

bool IS_TO_PSEUDO_COLOR = true;
bool IS_TO_TEMP = true;
image_transport::Publisher image_pub;
palette palet;


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
#if ROS_VERSION_MINIMUM(ROS_MIN_MAJOR, ROS_MIN_MINOR, ROS_MIN_PATCH)
#else
  sensor_msgs::CvBridge bridge;
#endif
  try {

    // get image

#if ROS_VERSION_MINIMUM(ROS_MIN_MAJOR, ROS_MIN_MINOR, ROS_MIN_PATCH)
    cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(msg, "mono16");
    cv::Mat img = ptr->image;
#else
    cv::Mat img = bridge.imgMsgToCv(msg, "mono16");
#endif
    //convert 8 bit and false color conversion
    
    cv::Mat img_mono8_ir, img_mono8;
    img_mono8_ir.create(img.rows, img.cols, CV_8UC1);

    converter_16_8::Instance().convert_to8bit(img, img_mono8_ir, IS_TO_TEMP);
    //converter_16_8::Instance().toneMapping(img, img_mono8_ir);
    sensor_msgs::ImagePtr linshi_msg;
    if (IS_TO_PSEUDO_COLOR) {
      // img_mono8 to CV_8UC3
      convertFalseColor(img_mono8_ir, img_mono8, palet, IS_TO_TEMP,
                        converter_16_8::Instance().getMin(), converter_16_8::Instance().getMax());
      linshi_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_mono8).toImageMsg();
    } else {
      linshi_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_mono8_ir).toImageMsg();
    }
    // 除bgr8之外，还有mono16、mono8代表单通道
    image_pub.publish(linshi_msg);
  }

#if ROS_VERSION_MINIMUM(ROS_MIN_MAJOR, ROS_MIN_MINOR, ROS_MIN_PATCH)
  catch (cv_bridge::Exception& e)
#else
  catch (sensor_msgs::CvBridgeException& e)
#endif
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv) {
  // if (argc != 3){ROS_ERROR("need IS_TO_PSEUDO_COLOR, IS_TO_TEMP as argument"); return -1;}
  
  ros::init(argc, argv, "ir_convert");
  ros::NodeHandle nh;
  // ////////////////////////////////////////////////

  bool is_temp, is_color;
  std::string pallet_choice;

  nh.getParam("/ir_convert_node/color/IS_TO_PSEUDO_COLOR", is_color);
  ROS_INFO("is_color = ", is_color);
  nh.getParam("/ir_convert_node/temp/IS_TO_TEMP", is_temp);
  ROS_INFO("is_temp = ", is_temp);
  nh.getParam("/ir_convert_node/color/palette", pallet_choice);
  ROS_INFO("pallet_choice = %s", pallet_choice);
  
  if (is_color){IS_TO_PSEUDO_COLOR = true; ROS_INFO("to pseudo image");}
  else {IS_TO_PSEUDO_COLOR = false;ROS_INFO("to temperature map");}
  if (is_temp) {IS_TO_TEMP = true; ROS_INFO("to TEMP");}
  else{IS_TO_TEMP = false;ROS_INFO("not to temp");}
  palet = GetPalette(pallet_choice);
  // /////////////////////////////////////////////////////
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 10, imageCallback);
  image_pub = it.advertise("/thermal/pseudo_color", 10);    
  
  ros::spin();

  return 0;
}
