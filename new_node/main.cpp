#include <ros/ros.h> 
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include "../image_convert/image_converter.h"

#include "ros/ros.h"
#include "std_msgs/String.h"


ros::Publisher image_pub ;

void chatterCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr  cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
//    cv_ptr->image.copyTo(cv_camera_);
    cv::Mat image_gray;
    cv::cvtColor(cv_ptr->image, image_gray,CV_BGR2GRAY);//灰度化
	
    cv_bridge::CvImage  cvi;
    sensor_msgs::Image  ros_img;
    ros::Time time=ros::Time::now();
    cvi.header.stamp = time;
    cvi.header.frame_id = "image";
    cvi.encoding = "mono8";
    cvi.image = image_gray;
    cvi.toImageMsg(ros_img);
    image_pub.publish(cvi.toImageMsg());
 
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "img_tran");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 1000, chatterCallback);
  image_pub = n.advertise<sensor_msgs::Image>("/mono_image", 1000);

  ros::spin();

  return 0;
}
