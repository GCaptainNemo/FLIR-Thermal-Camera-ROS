#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv/highgui.h>
#include <unistd.h>
#include <stdio.h>
#include<sys/types.h>
#include<dirent.h>

#define ROS_MIN_MAJOR 1
#define ROS_MIN_MINOR 8
#define ROS_MIN_PATCH 16

#if ROS_VERSION_MINIMUM(ROS_MIN_MAJOR, ROS_MIN_MINOR, ROS_MIN_PATCH)
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/CvBridge.h>
#endif



cv_bridge::CvImagePtr cv_ptr;
cv::Mat out_img;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    out_img = cv_ptr->image.clone();
    cv::Size img_size = out_img.size();
    ROS_INFO("cv_ptr->image.size = %d, %d", img_size.height, img_size.width);
    return;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    exit(0);
  }
}

int IsFolderExist(const char* path)
{
    DIR *dp;
    if ((dp = opendir(path)) == NULL)
    {
        return 0;
    }
 
    closedir(dp);
    return -1;
}

int IsFileExist(const char* path)
{
    return !access(path, F_OK);
}

std::string getFileName(const std::string & save_file_dir)
{
  ROS_INFO("in getFileName!");
      
  int index = 1;
  char name[5];
  std::string file_name;
  while (true)
  {
      sprintf(name, "%04d",index);
      ROS_INFO("name = %s", name);
      file_name = save_file_dir + std::string(name) + std::string("_ir.jpg");
      ROS_INFO("file_name = %s", file_name.c_str());
      if (IsFileExist(file_name.c_str()))
        {index+=1;}
      else
        break; 

  }
  ROS_INFO("name = %s", file_name.c_str());
  return file_name;
}

int main(int argc, char **argv) {
  // if (argc != 2){ROS_INFO("please enter output file name!!"); return -1;}
  // std::string file_dir = "/home/why/ROS_others/ws_FLIR_tools/data/";
  
  // std::string filename(argv[1]);
  ros::init(argc, argv, "ir_snapshot");
  ros::NodeHandle nh;
  std::string save_dir;
  nh.getParam("save_dir",save_dir);
  ROS_INFO("IR snapshot node save_dir = %s", save_dir.c_str());
  if(!IsFolderExist(save_dir.c_str())){ROS_ERROR("save dir doesn't exist"); return -1;}
  ROS_INFO("save dir exists!!");
  
  std::string file_dir = getFileName(save_dir);
  ROS_INFO("IR snapshot node file_dir = %s", file_dir.c_str());

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 10, imageCallback);
  while (true)
  {
    cv::Size img_size = out_img.size();
    if (img_size.width != 0){break;}
    ros::spinOnce();
  }
  cv::imwrite(file_dir, out_img);
  return 0;
}

