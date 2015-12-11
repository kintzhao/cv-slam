#ifndef VELCONTROL_H
#define VELCONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  // for velocity commands
#include <geometry_msgs/TwistStamped.h>  // for velocity commands

#include <nav_msgs/Odometry.h>
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#define CV_PI   3.1415926535897932384626433832795

#include <iostream>
#include <stdlib.h>

using namespace std ;

typedef struct roPos
{
   double  x;
   double  y;
   double  theta;
   void init(double x1, double x2, double x3)
   {
     x = x1;
     y = x2;
     theta = x3;
   }
} roPos;

class velControl
{
public:
    velControl();
    roPos  start_pos_;
    roPos  curr_pos_;


protected:

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

private:
    double rect_width_;
    double rect_height_;
    double offset_xy_;
    double offset_theta_;

    double vel_line_;
    double vel_angle_;

    int state_;
    bool Is_Fininsh_;

    ros::NodeHandle handle_;
    ros::Publisher  vel_pub_ ;
    ros::Subscriber sub_ ;
    geometry_msgs::Twist  controlVel_;

private:
    double angleWrap(double angle);
    bool init();
    bool velocityControl();

    int vel_pub_speed_;

};

#endif // VELCONTROL_H
