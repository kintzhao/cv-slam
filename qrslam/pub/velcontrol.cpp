#include "velcontrol.h"


velControl::velControl()
{
    init();
    vel_pub_ = handle_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
    sub_ = handle_.subscribe("/odom", 1, &velControl::odomCallback,this);
}
bool velControl::init()
{
    start_pos_.init(0.0, 0.0, 0.0);
    curr_pos_.init(0.0, 0.0, 0.0);
    rect_width_ = 1.0;
    rect_height_ = 1.0;
    offset_xy_ = 0.0001;
    offset_theta_ = 1.0/360*CV_PI;
    vel_line_ = 0.33;
    vel_angle_ = 0.2;


    vel_pub_speed_ = 5;

    state_ = 0;
    Is_Fininsh_ = true;
}
void velControl::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    static int count = 0;
    geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);
    curr_pos_.x = msg->pose.pose.position.x;
    curr_pos_.y = msg->pose.pose.position.y;
    curr_pos_.theta = yaw;
    if(Is_Fininsh_)
    {
        start_pos_.x = curr_pos_.x;
        start_pos_.y = curr_pos_.y;
        start_pos_.theta = curr_pos_.theta;

    }
    count++;
    if( count %= vel_pub_speed_ )
    {
        velocityControl();
    }

}
bool velControl::velocityControl()
{
    geometry_msgs::Twist controlVel_;
    switch (state_)
    {
    case 0: //前进1米
        if ( abs(curr_pos_.x - start_pos_.x) < rect_width_ )
        {
            cout<<"0 curr_pos_x  "<<abs(curr_pos_.x - start_pos_.x) <<endl;
            controlVel_.angular.z = 0.0;
            controlVel_.linear.x  = vel_line_;
            controlVel_.linear.y  = 0;
            controlVel_.linear.z  = 0;
            Is_Fininsh_ = false;
        }
        else
        {
            cout<<"0 curr_pos_x  "<<abs(curr_pos_.x - start_pos_.x) <<endl;
            controlVel_.angular.z = 0.0;
            controlVel_.linear.x  = 0.0;
            controlVel_.linear.y  = 0;
            controlVel_.linear.z  = 0;
            state_ = 1;
            Is_Fininsh_ = true;
        }
        break;
    case 1: //原地转90°
        if ( abs(angleWrap(curr_pos_.theta - start_pos_.theta)) < CV_PI/2 )
        {
            cout<<"1 curr_pos_theta  "<<curr_pos_.theta - start_pos_.theta<<endl;
            controlVel_.angular.z = vel_angle_;
            controlVel_.linear.x  = 0;
            controlVel_.linear.y  = 0;
            controlVel_.linear.z  = 0;
            Is_Fininsh_ = false;
        }
        else
        {
            cout<<"1 curr_pos_theta  "<<curr_pos_.theta - start_pos_.theta<<endl;
            ROS_INFO(" %d ",curr_pos_.theta - start_pos_.theta);
            controlVel_.angular.z = 0.0;
            controlVel_.linear.x  = 0.0;
            controlVel_.linear.y  = 0;
            controlVel_.linear.z  = 0;
            state_ = 2;
            Is_Fininsh_ = true;
        }
        break;
    case 2: //前进0.5米
        if ( abs(curr_pos_.y - start_pos_.y ) < rect_height_ )
        {
            cout<<"2 curr_pos_y  "<<abs(curr_pos_.y - start_pos_.y ) <<endl;
            controlVel_.angular.z = 0.0;
            controlVel_.linear.x  = vel_line_;
            controlVel_.linear.y  = 0;
            controlVel_.linear.z  = 0;
            Is_Fininsh_ = false;
        }
        else
        {
            cout<<"2 curr_pos_y  "<<abs(curr_pos_.y - start_pos_.y ) <<endl;
            controlVel_.angular.z = 0.0;
            controlVel_.linear.x  = 0.0;
            controlVel_.linear.y  = 0;
            controlVel_.linear.z  = 0;
            state_ = 3;
            Is_Fininsh_ = true;
        }
        break;
    case 3: //原地转90°
        if ( abs(angleWrap(curr_pos_.theta - start_pos_.theta)) < CV_PI/2 )
        {
            cout<<"3 curr_pos_theta  "<< abs(angleWrap(curr_pos_.theta - start_pos_.theta)) <<endl;
            controlVel_.angular.z = vel_angle_;
            controlVel_.linear.x  = 0;
            controlVel_.linear.y  = 0;
            controlVel_.linear.z  = 0;
            Is_Fininsh_ = false;
        }
        else
        {
            cout<<"3 curr_pos_theta  "<<abs(angleWrap(curr_pos_.theta - start_pos_.theta)) <<endl;
            controlVel_.angular.z = 0.0;
            controlVel_.linear.x  = 0.0;
            controlVel_.linear.y  = 0;
            controlVel_.linear.z  = 0;
            state_ = 4;
            Is_Fininsh_ = true;
        }
        break;
    case 4: //前进1米
        if ( abs(curr_pos_.x - start_pos_.x) < rect_width_ )
        {
            cout<<"4 curr_pos_x  "<<abs(curr_pos_.x - start_pos_.x) <<endl;
            controlVel_.angular.z = 0.0;
            controlVel_.linear.x  = vel_line_;
            controlVel_.linear.y  = 0;
            controlVel_.linear.z  = 0;
            Is_Fininsh_ = false;
        }
        else
        {
            cout<<"4 curr_pos_x  "<<abs(curr_pos_.x - start_pos_.x) <<endl;
            controlVel_.angular.z = 0.0;
            controlVel_.linear.x  = 0.0;
            controlVel_.linear.y  = 0;
            controlVel_.linear.z  = 0;
            state_ = 5;
            Is_Fininsh_ = true;
        }
        break;
    case 5: //原地转90°
        if ( abs(angleWrap(curr_pos_.theta - start_pos_.theta)) < CV_PI/2 )
        {
            cout<<"5 curr_pos_theta  "<<abs(angleWrap(curr_pos_.theta - start_pos_.theta)) <<endl;
            controlVel_.angular.z = vel_angle_;
            controlVel_.linear.x  = 0;
            controlVel_.linear.y  = 0;
            controlVel_.linear.z  = 0;
            Is_Fininsh_ = false;
        }
        else
        {
            cout<<"5 curr_pos_theta  "<<abs(angleWrap(curr_pos_.theta - start_pos_.theta)) <<endl;
            controlVel_.angular.z = 0.0;
            controlVel_.linear.x  = 0.0;
            controlVel_.linear.y  = 0;
            controlVel_.linear.z  = 0;
            state_ = 6;
            Is_Fininsh_ = true;
        }
        break;
    case 6: //前进0.5米
        if ( abs(curr_pos_.y - start_pos_.y) < rect_height_ )
        {
            cout<<"6 curr_pos_y  "<<abs(curr_pos_.y - start_pos_.y) <<endl;
            controlVel_.angular.z = 0.0;
            controlVel_.linear.x  = vel_line_;
            controlVel_.linear.y  = 0;
            controlVel_.linear.z  = 0;
            Is_Fininsh_ = false;
        }
        else
        {
            cout<<"6 curr_pos_y  "<<abs(curr_pos_.y - start_pos_.y) <<endl;
            controlVel_.angular.z = 0.0;
            controlVel_.linear.x  = 0.0;
            controlVel_.linear.y  = 0;
            controlVel_.linear.z  = 0;
            state_ = 7;
            Is_Fininsh_ = true;
        }
        break;
    case 7: //原地转90°
        if ( abs(angleWrap(curr_pos_.theta - start_pos_.theta)) < CV_PI/2 )
        {
            cout<<"7 curr_pos_theta  "<<abs(angleWrap(curr_pos_.theta - start_pos_.theta))  <<endl;
            controlVel_.angular.z = vel_angle_;
            controlVel_.linear.x  = 0;
            controlVel_.linear.y  = 0;
            controlVel_.linear.z  = 0;
            Is_Fininsh_ = false;
        }
        else
        {
            cout<<"7 curr_pos_theta  "<<abs(angleWrap(curr_pos_.theta - start_pos_.theta))  <<endl;
            controlVel_.angular.z = 0.0;
            controlVel_.linear.x  = 0.0;
            controlVel_.linear.y  = 0;
            controlVel_.linear.z  = 0;
            state_ = 0;
            Is_Fininsh_ = true;
        }
        break;

    default:
        controlVel_.angular.z = 0.0;
        controlVel_.linear.x  = 0.0;
        controlVel_.linear.y  = 0;
        controlVel_.linear.z  = 0;
        Is_Fininsh_ = true;
        break;
    }
    vel_pub_.publish(controlVel_);
}
double velControl::angleWrap(double angle)
{
    ///这个函数用来实现把角度规划到-pi至pi之间
    if (angle>=CV_PI)
        while (angle >= CV_PI)
        { angle = angle-2*CV_PI;}
    else if (angle < -1.0*CV_PI)
        while (angle < -1.0*CV_PI)
        { angle = angle+2*CV_PI;}
  return angle;
}
int main(int argc,char** argv)
{
    ros::init(argc,argv,"odompub");
    velControl odom_vel_;
    ros::spin();
    return 0;
}











