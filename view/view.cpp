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

#include <cv_slam/odom.h>
#include <cv_slam/landmark.h>
#include <cv_slam/robot.h>
#include <cv_slam/systemState.h>
using namespace std;
using namespace cv;

void showOdom(cv::Mat& map, const cv_slam::odom rob_odom, const Scalar rgb);
void showRobot(cv::Mat& map, const cv_slam::robot robot, const Scalar rgb);
void drawCoordinate(cv::Mat& mat);
std::string   int2str(int num);
void showLandmarks(cv::Mat& map, const vector<cv_slam::landmark> landmarks, const Scalar rgb);
void chatterCallback(const cv_slam::systemStateConstPtr& msg);


std::string   float2str(float num);
void showOdom(Mat image, const cv_slam::odom rob_odom, const Scalar rgb, const int x_coordinate, const int y_coordinate);
void showRobot(Mat image, const cv_slam::robot rob_odom, const Scalar rgb, const int x_coordinate, const int y_coordinate);


ros::Publisher image_pub_ ;
ros::Publisher state_pub_ ;
Mat map_;
Mat view_;
Mat state_img_;

const int map_width_ = 2000;// = 150 ;
const int map_height_ = 2000 ;// = 100 ;
const int map_base_x_ = 1000;// = 150 ;
const int map_base_y_ = 1000 ;// = 100 ;
const float map_scale_ = 0.25;           //缩放倍数
Size   map_size_;              //目标图像尺寸

int main(int argc, char **argv)
{
  ros::init(argc, argv, "view");
  ros::NodeHandle n;
  map_ =  cv::imread("./OutPut/map/map.png", CV_LOAD_IMAGE_COLOR); //for display

  map_size_.width  = map_.cols * map_scale_;
  map_size_.height = map_.rows * map_scale_;
  resize(map_, state_img_, Size(640,480));

  cv::line(map_,cv::Point(map_base_x_, map_base_y_),cv::Point(map_width_, map_base_y_),CV_RGB(255,0,0),2,8); //x
  cv::line(map_,cv::Point(map_base_x_, map_base_y_),cv::Point(map_base_x_, 0),CV_RGB(0,255,0),2,8); //y

  std::string text ="X";
  cv::putText(map_,text,Point(map_width_-100 ,map_base_y_),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(255,0,0)); //x
  text ="Y";
  cv::putText(map_,text,Point(map_base_x_, 100),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,255,0)); //y



  ros::Subscriber sub = n.subscribe("/systemStates", 100, chatterCallback);
  image_pub_ = n.advertise<sensor_msgs::Image>("/view", 100);
  state_pub_ = n.advertise<sensor_msgs::Image>("/state_img", 100);

  ros::spin();
  return 0;
}

void chatterCallback(const cv_slam::systemStateConstPtr& msg)
{
    cv_slam::odom   odom = msg->odom;
    cv_slam::robot  robot = msg->robot_state;
    vector<cv_slam::landmark>   landmarks = msg->landmark;

    showOdom(map_, odom, CV_RGB(0,0,0)) ;
    showRobot(map_, robot, CV_RGB(0,0,255));
    drawCoordinate(map_);
    map_.copyTo( view_ );    //避免历史重影 landmark

    showLandmarks(view_, landmarks, CV_RGB(0,0,0));
    resize(view_, view_, map_size_);

    cv_bridge::CvImage  cvi;
    sensor_msgs::Image  ros_img;
    ros::Time time=ros::Time::now();
    cvi.header.stamp = time;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    cvi.image = view_;//Mat
    cvi.toImageMsg(ros_img);
    image_pub_.publish(cvi.toImageMsg());

    Mat state_show;
    state_img_.copyTo( state_show );    //避免历史重影 landmark

    showOdom(state_show, odom, CV_RGB(0,0,0),50,50);
    showRobot(state_show, robot, CV_RGB(0,0,0),300,50);

    cv_bridge::CvImage  cvi_state;
    sensor_msgs::Image  state_img;
    ros::Time time2=ros::Time::now();

    cvi_state.header.stamp = time2;
    cvi_state.header.frame_id = "image";
    cvi_state.encoding = "bgr8";
    cvi_state.image = state_show; //Mat
    cvi_state.toImageMsg(state_img);
    state_pub_.publish(cvi_state.toImageMsg());
}

void showOdom(cv::Mat& map, const cv_slam::odom rob_odom, const Scalar rgb)
{
    Point start, end;
    start.x =  rob_odom.x + map_base_x_;
    start.y = -rob_odom.y + map_base_y_ ;

    end.x = rob_odom.x + 200 * cos(rob_odom.theta);
    end.y = rob_odom.y + 200 * sin(rob_odom.theta);  //display  y  convert ..

    end.x =  end.x + map_base_x_;
    end.y = -end.y + map_base_y_ ;

    circle(map,start,4,rgb,4,8 );
    line( map,start,end,rgb,2,8 );
}
void showRobot(cv::Mat& map, const cv_slam::robot robot, const Scalar rgb)
{
    int temp_X = robot.x + map_base_x_;
    int temp_Y = robot.y + map_base_y_;
    temp_Y = map.rows - temp_Y ;
    cv::circle(map,Point( temp_X,temp_Y),2,rgb,1); //绘制 robot
}
void drawCoordinate(cv::Mat& mat)
{
    std::string text ="Y";
    cv::putText(mat,text,Point(20,20),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    cv::line(mat,cv::Point(1,1),cv::Point(1,mat.rows),CV_RGB(255,0,0),1,8);

    text ="O";
    cv::putText(mat,text,Point(20,mat.rows-20),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));

    text ="X";
    cv::putText(mat,text,Point(mat.cols-20,mat.rows-20),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    cv::line(mat,cv::Point(1,mat.rows-1),cv::Point(mat.cols,mat.rows-1),CV_RGB(255,0,0),1,8);

}
void showLandmarks(cv::Mat& map, const vector<cv_slam::landmark> landmarks, const Scalar rgb)
{
    for (int t = 0; t < landmarks.size(); t++)
    {
        cv_slam::landmark lanmark = landmarks.at(t);

        float X= lanmark.x + map_base_x_;
        float Y= -lanmark.y + map_base_y_;
        cv::circle(map,Point( X,Y),2,rgb,2); //绘制mark位置

        Point start, end;
        start.x =  lanmark.x + map_base_x_;
        start.y = -lanmark.y + map_base_y_ ;

        end.x = lanmark.x + 200 * cos(lanmark.theta);
        end.y = lanmark.y + 200 * sin(lanmark.theta);  //display  y  convert ..
        end.x =  end.x + map_base_x_;
        end.y = -end.y + map_base_y_ ;
        line( map,start,end,rgb,2,8 );

        //cv::Mat landmark_cov = getCovMatrix(miu_state, 3+t*3, 4+t*3);
        //cv::Point2f center(X,Y);
        // drawEllipse(map, center, landmark_cov, CV_RGB(0, 150,0) );

        std::string text = int2str(lanmark.id);
        cv::putText(map,text,Point(X,Y+20),CV_FONT_HERSHEY_COMPLEX, 2, CV_RGB(255, 0,0) );
    }

}
void showOdom(Mat image, const cv_slam::odom odom, const Scalar rgb, const int x_coordinate, const int y_coordinate)
{
    const int ROBOT_DEFAULT_ARROW_LEN = 30;

    Point3d robot_pose;
    robot_pose.x = odom.x;
    robot_pose.y = odom.y;
    robot_pose.z = odom.theta;
    //采用地图固定下  初始坐标系调整就可以
    Point start, end;
    start.x = x_coordinate;
    start.y = y_coordinate;

    int thickness = 1;
    int lineType = 8;
    line( image, start, start+Point(500,0), CV_RGB(0,255,0), 1, lineType );  //  x轴
    line( image, start, start+Point(0,500), CV_RGB(0,155,0), 1, lineType );  //  y轴

    circle(image, start,3, rgb, 2, lineType );
    end.x = start.x + ROBOT_DEFAULT_ARROW_LEN * cos(robot_pose.z);  //放大5倍
    end.y = start.y - ROBOT_DEFAULT_ARROW_LEN * sin(robot_pose.z);  //display  y  convert
    line( image, start, end, rgb, thickness, lineType );

    //  标记坐标信息
    std::string text_id ="x: "+ float2str( robot_pose.x  );
    cv::putText(image, text_id, Point(50,150), CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(0,0,255));
    std::string text1 ="y: "+ float2str( robot_pose.y );
    cv::putText(image,text1, Point(50,200), CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(0,0,255));
    std::string text2 ="z: "+ float2str( robot_pose.z*180/ 3.14159 );
    cv::putText(image, text2, Point(50,250), CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(0,0,255));
}
void showRobot(Mat image, const cv_slam::robot robot, const Scalar rgb, const int x_coordinate, const int y_coordinate)
{
    const int ROBOT_DEFAULT_ARROW_LEN = 30;

    Point3d robot_pose;
    robot_pose.x = robot.x;
    robot_pose.y = robot.y;
    robot_pose.z = robot.theta;

    Point start, end;
    start.x = x_coordinate;
    start.y = y_coordinate;

    int thickness = 1;
    int lineType = 8;
    line( image,start,start+Point(500,0),CV_RGB(0,255,0),1,lineType );  //  x轴
    line( image,start,start+Point(0,500),CV_RGB(0,155,0),1,lineType );  //  y轴

    circle(image,start,3,rgb,2,lineType );
    end.x = start.x + ROBOT_DEFAULT_ARROW_LEN * cos( robot_pose.z );
    end.y = start.y - ROBOT_DEFAULT_ARROW_LEN * sin( robot_pose.z );  //display  y  convert
    line( image,start,end,rgb,thickness,lineType );

    //  标记坐标信息
    std::string text_id ="x: "+ float2str( robot_pose.x  );
    cv::putText(image,text_id,Point(300,150),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(255,0,0));
    std::string text1 ="y: "+ float2str( robot_pose.y );
    cv::putText(image,text1,Point(300,200),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(255,0,0));
    std::string text2 ="z: "+ float2str( robot_pose.z *180/ 3.14159);
    cv::putText(image,text2,Point(300,250),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(255,0,0));

}

std::string   int2str(int num)
{
    std::stringstream ss;
    ss  <<  num;
    std::string text = ss.str();
    return text;
}
std::string   float2str(float num)
{
    std::stringstream ss;
    ss << num;
    std::string text = ss.str();
    return text;
}
