#ifndef QRSLAM_H
#define QRSLAM_H

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/timeb.h>

#include <cstring>
#include <cstdio>
#include <math.h>
#include <iostream>
#include <vector>
#include <map>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <visualization_msgs/Marker.h>
#include <cmath>


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <cv_slam/odom.h>
#include <cv_slam/landmark.h>
#include <cv_slam/robot.h>
#include <cv_slam/systemState.h>

//#include "simulator.h"  //  g2o相关的库

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <fstream>

#include "./class/detectqrcode.h"
#include "../image_convert/image_converter.h"


//================== g2o =================
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam2d/types_slam2d.h"

using namespace g2o;

typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
//================== /g2o ================

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Image> slamSyncPolicy;

using ARToolKitPlus::TrackerSingleMarker;
using ARToolKitPlus::ARMarkerInfo;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

typedef struct Point3ffi
{
    float x;
    float y;
    int z;
    void init(float x1,float y1,int z1)
    {
        x=x1;
        y=y1;
        z=z1;
    }
}Point3ffi;

typedef struct Point4fffi
{
    float x;
    float y;
    float theta;
    int id;
    void init(float x1,float y1, float theta1, int id1)
    {
        x=x1;
        y=y1;
        theta=theta1;
        id = id1;
    }
}Point4fffi;


typedef struct OdomMessage
{
    ros::Time  stamp;
    double v;
    double w;
    double theta;
    double x;
    double y;
    void init(double x0,double y0,double theta0,double v0,double w0, ros::Time  stamp0  )
    {
        x = x0;
        y = y0;
        theta = theta0;
        v = v0;
        w = w0;
        stamp = stamp0;
    }
    void init(const OdomMessage odom)
    {
        x = odom.x;
        y = odom.y;
        theta = odom.theta;
        v = odom.v;
        w = odom.w;
        stamp = odom.stamp;
    }
}OdomMessage;

typedef struct landmarkObservation
{
    int id;
    Point2f measure;
    Point2f predict;
    Point3f robot_odom;
    Point3f robot_predict;
    ros::Time  stamp;
}landmarkObservation;

typedef struct PicInf
{
    int id;
    int u;
    int v;
}PicInf;

typedef struct PicNode
{
    int num;
    vector<PicInf>  infs;
}PicNode;

typedef struct RobNode
{
    int id;
    Point3f robot_pos;
    Point3f robot_pos_odom;
    PicNode pic_node;
}RobNode;

typedef struct SearchInfo
{
    int num_marks;
    int num_global_try;
    int num_global;
    int num_local_try;
    int num_local;

    float time_global;
    float time_local;
    void init(int marks, int global, int gTry, int local, int lTry, float gTime, float lTime)
    {
      num_marks = marks;

      num_global = global;
      num_global_try = gTry;

      num_local = local;
      num_local_try = lTry;

      time_global = gTime;
      time_local = lTime;
    }
}SearchInfo;


using namespace std;
using namespace cv;
class CVSlam
{
public :
    CVSlam(char* addr, char *config);
    ~CVSlam();
    void readConfigure(char* config);

    //**********与机器人位姿相关的变量****************************
    //------------与机器人定位相关的 variables--------------//

    vector<Point2f> velocities_;          //For velocity model,用来保存直线和角输入,其保存的第一个元素为零，第二个元素代表从第一个位置与第二个位置之间的速度，以此类推
    vector<Point2f> odom_velocities_;          //For velocity model,用来保存直线和角输入,其保存的第一个元素为零，第二个元素代表从第一个位置与第二个位置之间的速度，以此类推

    vector<Point3ffi> observations_;

    vector<Point4fffi> current_marks_observed_;

    vector<Point3f> est_path_points_;       //估计所得路点
    vector< vector<Point3f> > landmarks_system_;       //估计所得路点

    ///////////////////////EKFSLAM变量
    Mat miu_state_ ;           //EKF_SLAM中组合状态向量
    Mat miu_convar_p_ ;           //EKF_SLAM中的组合方差矩阵

    vector<int> id_all_marks_;
    vector<int> index_landmarks_;

    const int map_width_ = 2000;// = 150 ;
    const int map_height_ = 2000 ;// = 100 ;
    float map_scale_;           //缩放倍数
    Size   map_size_;              //目标图像尺寸

    Mat map_;

    int map_base_x_ = 1000;// = 150 ;
    int map_base_y_ = 1000 ;// = 100 ;

    float a1_;//= 0.001;//0.1;
    float a2_;//= 0;//0.1;
    float a3_;//= 0;//0.1;
    float a4_;//= 0.005;//0.1;

    float sigma_r_  ;//= 0.1;
    float sigma_phi_;//= 0.1;   //观测模型噪声
    float p_init_x_;//= 0.0001;        // 0.1;//0.1;//1;//100;//0.1;
    float p_init_y_;//= 0.01;        //0.10;//0.1;//1;//100;//0.1;
    float p_init_theta_ ;//= 0.02618;    //0.38;//0.1;//0.38;

    float convar_x_;// = 0.0005;        // 0.1;//0.1;//1;//100;//0.1;
    float convar_y_;// = 0.0005;        //0.10;//0.1;//1;//100;//0.1;
    float convar_theta_;// = 0.000685;    //0.38;//0.1;//0.38;
    float convar_measure0_;
    float convar_measure1_;
    float convar_measure2_;
    float convar_measure3_;// { 0.1, 0.0, 0.0, 0.05}
    float convar_measure4_;

    //     const float update_odom_linear_ = 4;
    float update_odom_linear_;// = 0.5;
    float update_odom_angle_;//  = 0.17;
    float stamp_interval_;// = 0.005;//0.002;

    int INIT_LOCALIZATION_ = 10;// 初始定位缓存大小
    int Fixed_mark_ID_ = 0; // 用作初始定位的landmark id .
    RNG   rng;

    ros::Time time_bag_,time_bag_old;

    float angleWrap(float& angle);
    float   genGaussianValue(float Sigma2);   //generate a Gaussian value with variance sigma2
    void    displayMatrix(Mat matrix) ;
    void    writeMatrix(Mat matrix, ofstream &f_test) ;

    int visual_mark_num_;

public:
    vector<OdomMessage>  odom_messages_vector;
    vector<CPointsFour> mark5_init_vector_;

    ros::NodeHandle   ar_handle;
    message_filters::Subscriber<nav_msgs::Odometry>* odom_sub_ ;             // topic1 输入
    message_filters::Subscriber<sensor_msgs::Image>* img_sub_;   // topic2 输入
    //    TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Image>* sync_wiki_;       // 同步

    message_filters::Synchronizer<slamSyncPolicy>* sync_;
    image_transport::ImageTransport transport_;
    image_transport::Subscriber qr_image_sub_;

    tf::TransformBroadcaster odom_to_img_;
    tf::TransformListener listener_;
    ros::Publisher map_pose_publisher_;
    ros::Publisher robot_pose_publisher_;
    ros::Publisher systemStates_;


   // vector<geometry_msgs::PoseStamped>   poses_map;
    ros::Publisher marker_pub_;

    DetctQrcode*  pQrDetect_;
   // DetctQrcode*  pLocalQrDetect_;

    //public function
    std::string  int2str(int num);
    std::string  float2str(float num) ;

    int   landmark_num_ ;

    //display
    ImageConverter* raw_img_cvt_ = NULL;
    ImageConverter* slam_img_cvt_ = NULL;
    ImageConverter* cut_img_= NULL;

    cv::Mat raw_global_map_ ;
    cv::Mat cv_camera_;

    void showRobot(Mat &map, const OdomMessage rob_odom, const Scalar rgb);

    void drawCoordinate(cv::Mat& mat);
    void storeData(void );
    bool is_odom_update_ ;
    bool is_img_update_;


    //coordinate change
    float coordinate_x_;
    float coordinate_y_;
    float coordinate_angle_;

    Point3f diffCoordinate(CPointsFour mark_2d,CPointsFourWorld mark_2d_world);
    void  showRobotTriangle(cv::Mat& map, OdomMessage rob_odom, Scalar rgb);


    void storeSystemState( Mat systemState);

    void stateExtended(const int curr_num, const int num_old);
    ros::Time odom_stamp_;
    ros::Time odom_stamp_old_;
    ros::Time image_stamp_;
    ros::Time odom_init_stamp; //初始下的时间戳

    int  Fixed_landmark_index_; //用于初始单位的landmark位于系统状态landmark的次序
    cv::Mat global_map_display_; //析构的时候可以保存图片

    int num_EKFSlam_predict_;  //记录预测次数
    int num_EKFSlam_update_;  //记录更新次数
    int num_time_interval;
    vector< vector<landmarkObservation> > landmark_vectors_;
    vector<Point2f> K_vec;
    vector< vector<Point2f> >  Kt_vectors_;
    OdomMessage robot_odom_;

    CPointsFourWorld mark_2d_world_ ;

    vector<Point3f>  state_v_;
    vector< vector<Point3f> > states_vv_;


//=========================
    Mat image_data_;

    Point3f best_mark_center_;  // (u,v,id)
    Point3f best_local_mark_center_;
    int best_mark_center_side_;
    int best_mark_center_id_;

    vector<RobNode> robot_nodes_;
    Point3f last_rob_pose_;
    Point3f init_frame_diff_;
    bool  Is_Global_search_;
    SearchInfo search_info_;

    bool addInitVectorFull();

    void savePixDatas( vector<CPointsFour> vector_data );
    void saveMeterDatas(vector<CPointsFour> vector_data , ofstream &writeData);

    Point3f odomModel(Point2f motion, Point3f robotPose,float delta_time);
    void getOdomDataByAcc(const nav_msgs::Odometry::ConstPtr& odomMsg);


//==============================================
    Mat getImgDataNew(const sensor_msgs::ImageConstPtr& imgMsg);
    void startSLAM(const nav_msgs::Odometry::ConstPtr& pOdom, const sensor_msgs::ImageConstPtr& pImg) ; //回调中包含多个消息;
    void ekfslamNew(OdomMessage rob_odom, Mat &img);
    Point3f motionModelNew(Point2f motion, Mat& system_state, Mat& system_state_convar, int variable_num, float delta_time);

    Point2f  worldToCamera(Point3f observe_increase_world);
    Point2f  cameraToImage(Point2f observe_increase_camera, int id);
    int  findLandmarkId(int id) ;
    cv::Mat  getCovMatrix(const cv::Mat state_matrix, int i, int j);
    void drawEllipse(cv::Mat img_test, cv::Point2f center, cv::Mat covMartix, const Scalar& color);
    OdomMessage odomToWorld(const OdomMessage src);
    OdomMessage odomToWorld(const OdomMessage src, const Point3f frame_diff);
    void getObservationsNew();
    vector<Point4fffi> getObservationsNew( vector<CPointsFour> marks_metric );
    Point4fffi addObservationPosNew(CPointsFour code_one, int id);
    void updateSLAM(Mat& system_state, Mat& system_state_convar, int variable_num, vector<Point4fffi>& curr_Marks, vector<int> &index_landmarks);
    int  getLandmarkNumNew(const vector<Point4fffi> curr_marks_observed, vector<int>& landmark_ids);
    void posesPublishNew();
    void saveSystemState(ofstream &file );
    void showImageNew();
    void showSystemStateRobotNew(cv::Mat& map, const Mat miu_state, const Scalar rgb);
    void showSystemStateLandmarkNew(cv::Mat& map, const Mat miu_state, const Scalar rgb);
    void showRobotOrientationNew(Mat image, OdomMessage rob_odom,Scalar rgb,int x_coordinate,int y_coordinate);
    void showRobotOrientationNew(Mat image, Mat rob_update_mat,Scalar rgb,int x_coordinate,int y_coordinate);
    bool frameFixedTransform(Mat& miu_state, const int fixed_mark_id);
    void systemTransforeNew(Mat& sys_state, Point3f trans);
    PicNode getPicInfo(const vector<CPointsFour>  marks_pix);
    int checkLoopClosure(const vector<RobNode> history_nodes,const PicNode current_pic_node, int num, int min_score_pic, int min_value_mark);
    int getMatchScore(PicNode history_pic_node, PicNode current_pic_node, int min_distance);
    bool evaluateNew(vector<CPointsFour> marks_pix, Point3f& best_mark_center);
    OdomMessage getOdomDataNew(const nav_msgs::Odometry::ConstPtr& odomMsg);
    bool addInitPos(cv::Mat img, const OdomMessage robot_state, const int Mark_id, const int num_full);
    vector<CPointsFour> pixToMetric(vector<CPointsFour> pix_vector);
    vector<CPointsFour> imgDetectMarks(cv::Mat image );
    vector<CPointsFour> imgDetectMarks(cv::Mat image, Point3f mark_center);
    void saveMarksPerImgPix( vector<CPointsFour> vector_data, ofstream&  file );
    void saveMarksPerImgMetric( vector<CPointsFour> vector_data,ofstream&  file );
    Point3f initFrameDiff(vector<CPointsFour> init_marks_metric, const CPointsFourWorld origin_world);
    void saveInitFrameDiff(const Point3f frame_diff, ofstream &file );
    cv::Mat inverseObservationModel(cv::Mat& full_img, Point3f& mark_center, Mat system_state);
    cv::Mat getSystemState(void );
    vector<CPointsFour>  imgProcessNew(Mat &img, bool& Is_Gobal_Search, Point3f& best_mark_info, SearchInfo& search_info);
    int getLandmarksNum();
    int  getFixedLandmarkIndex(const int fixed_mark_id);
    vector<int>  getLandmarksIndex();
    void  pubSystemStates();

//====================== g20 =========================
    void graphOptimization();  //回调中包含多个消息


    SparseOptimizer optimizer_;
    OptimizationAlgorithmGaussNewton* solver_;// = new OptimizationAlgorithmGaussNewton(blockSolver);
    void setG2oInfo(const Mat sys_state, const int index_g2o_node);
    Eigen::Matrix3d getRobotInformation();


};

#endif

