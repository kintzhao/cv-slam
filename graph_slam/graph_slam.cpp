/* read me :
 *1. the num of qrcode N :we init as the static..    --->>> the method of vector add.
 *
 *2.
 *
 *
*/
// ***ros 里程订阅的修改
//   标记  //rosOdom
#include "graph_slam.h"

ofstream state_miu("result.txt");
ofstream frobot("frobot.txt");
ofstream fmiu("miu.txt");
//ofstream fvel("vel.txt");
//ofstream fre("re.txt");   //　保存　cal_temp 预测累计量

ofstream fQrPixData("qr_pix_data.txt");
ofstream fQrMeterData("qr_meter_data.txt");
ofstream fQrMeterDataSide("qr_meter_data_side.txt");

//ofstream fIinitNum("init_num.txt");   // 用于调试初始化时数据缓冲量是否达到　CAMERA_FREQUENCE_DIFF_
ofstream fOdom("odom.txt");
ofstream fTest("test.txt");  //矩阵输出存放位置
//ofstream fVar("var.txt");  //矩阵输出存放位置
ofstream fobsevation("observation.txt");  //矩阵输出存放位置
ofstream fnewlandmark("newlandmark.txt");  //矩阵输出存放位置
ofstream fcoordinate_init("coordinate_init.txt");

ofstream ftimeStamp("time.txt");
ofstream fStamp("stamp.txt");
ofstream fStampAll("stampal.txt");
//ofstream fdebug("debug.txt");
ofstream fSearchCount("fSearchCount.txt");



graphSlam::graphSlam(char* addr,char* config):transport_( ar_handle)
{
    IS_FIRST_POS_ = true;
    index_rob_ = 1;
    total_mark_ = 20;
    index_rob_ += total_mark_;

    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    solver = new OptimizationAlgorithmGaussNewton(blockSolver);
    optimizer.setAlgorithm(solver);





    total_search_count_ = 0;
    global_search_count_ = 0;
    local_search_count_ = 0;
    FIRST_DETEDCTET_ = true;
    IS_GET_INIT_ODOM_COORDINATE_ = false;
    readConfigure(config);
    IS_SYSTEM_TRANSFER_ = false;
    map_lock_.x = 0.0;
    map_lock_.y = 0.0;
    map_lock_.z = 0.0;
    call_back = 0;
    ekf_start =0;
    ekf_end = 0;
    num_time_interval = 0;
    num_EKFSlam_predict_ = 0;
    num_EKFSlam_update_ = 0;
    Init_mark_in_systemState_j = -1;
    //    last_update_pose.x = 0.0;
    //    last_update_pose.y = 0.0;
    //    last_update_pose.z = 0.0;
    is_odom_update = false;
    is_img_update_  = false;
    data_filter_num = 0;
    landmark_have_drawed = 0;
    Vodom[DATA_FUSION_NUM] = {};
    Wodom[DATA_FUSION_NUM] = {};
    
    observed_mark_num_old = 0;

    coordinate_x_ = 0.0;
    coordinate_y_ = 0.0;
    coordinate_angle_ = 0.0;

    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(ar_handle, "/odom", 1);
    img_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(ar_handle, "/usb_cam/image_raw", 1);
    //sync_(odom_sub_, img_sub_, 10);         // 同步
    
    //    sync_wiki_ = new TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Image>(odom_sub_, img_sub_, 10);        // 同步
    //    sync_wiki_->registerCallback(boost::bind(&graphSlam::combineCallback,this, _1, _2));
    
    sync_ = new  message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(5), *odom_sub_, *img_sub_);
//    sync_->registerCallback(boost::bind(&graphSlam::combineCallback,this, _1, _2));
 //   sync_->registerCallback(boost::bind(&graphSlam::combineCallbackNew,this, _1, _2));
    sync_->registerCallback(boost::bind(&graphSlam::graphSlamCallBack,this, _1, _2));

    map_pose_publisher_ = ar_handle.advertise< geometry_msgs::PoseStamped >("map_pose", 1);
    robot_pose_publisher_ = ar_handle.advertise<nav_msgs::Odometry>("robot_pose", 1);
    marker_pub_ = ar_handle.advertise<visualization_msgs::Marker>("map_marker", 10);


    raw_img_cvt_   =  new ImageConverter("/slam/raw_flip_image");
    robot_img_cvt_ =  new ImageConverter("/slam/robot_image");
    slam_img_cvt_  =  new ImageConverter("/slam/graph_slam/slam_map");
    cut_img_  =  new ImageConverter("/cut_img");
    local_img_ =  new ImageConverter("/local_img");


    if ((raw_img_cvt_ == NULL) || (robot_img_cvt_ == NULL) || (slam_img_cvt_ == NULL))
    {
        exit(1);
    }
    
    raw_global_map_ =  cv::imread("./data/bl.png", CV_LOAD_IMAGE_COLOR); //for display
    if (! raw_global_map_.data )
    {
        ROS_INFO("Could not open or find the image ./data/bl.png");
    }
    else
    {
        cv::line(raw_global_map_,cv::Point(MAP_BASE_X_,0),cv::Point(MAP_BASE_X_,raw_global_map_.rows),CV_RGB(255,255,0),1,8);
        cv::line(raw_global_map_,cv::Point(0,MAP_BASE_Y_),cv::Point(raw_global_map_.cols,MAP_BASE_Y_),CV_RGB(255,255,0),1,8);
    }
    //××××××××××××××××相机配置信息 配置文件路径  addr  ;SELECT_MARK_FOR_INIT_用于初始定位的二维码id  ×××××××××××××××××××
    pQrDetect_ = new DetctQrcode(addr,SELECT_MARK_FOR_INIT_);
    pLocalQrDetect_ = new DetctQrcode(addr,SELECT_MARK_FOR_INIT_);
    if (pQrDetect_ == NULL)
    {
        exit(1);
    }
    delta_t_=0.0;

    IS_INIT_FINISH_ = false ;   //miu_SLAM的初始化标志，只有第一次进来时需要初始化

    robot_odom_.init(0.0,0.0,0.0,0.0,0.0);

    fQrPixData  << "size id c0x y c1x y c2x y c3x y center_x y " << endl;
    fQrMeterData  << "size id c0x y c1x y c2x y c3x y center_x y " << endl;
    fQrMeterDataSide  << "size id c0x y c1x y c2x y c3x y center_x y " << endl;
    fOdom << "x y theta dt v w " << endl;


    robot_odom_.init(0.0,0.0,0.0,0.0,0.0);
    odom_stamp_ = ros::Time::now();
    odom_stamp_old_ = ros::Time::now();

}

graphSlam::~graphSlam()
{
    ros::Time now_time = ros::Time::now();
    int name_time = (int)now_time.toSec() ;
    name_time = name_time % 31104000 ;
    int month,data,hour,minute,second;
    second = name_time % 60;
    name_time = name_time / 60;
    minute = name_time % 60;
    name_time = name_time /60;
    hour = name_time % 24;
    name_time = name_time/24;
    data = name_time % 30;
    name_time = name_time/30;
    month = name_time % 12;

    string file_name = "./" + float2str(month)+'-'+float2str(data)+"-"+float2str(hour)
            +"-"+float2str(minute)+"-"+float2str(second)+".png";

    cv::imwrite(file_name,global_map_display_);
    
    if (!raw_img_cvt_)
    {
        delete raw_img_cvt_;
    }
    if (!slam_img_cvt_)
    {
        delete slam_img_cvt_;
    }
    if (!pQrDetect_)
    {
        delete pQrDetect_;
    }
    cout<<"the map have been saved, program end!"<<endl;
}

// 读取EKFSLAM配置相应参数
void graphSlam::readConfigure(char* config)
{
    cv::FileStorage fileRead(config, cv::FileStorage::READ);

    fileRead["MAP_BASE_Y_"] >> MAP_BASE_Y_;
    fileRead["MAP_BASE_X_"] >> MAP_BASE_X_;

    fileRead["a1"] >> a1;
    fileRead["a2"] >> a2;
    fileRead["a3"] >> a3;
    fileRead["a4"] >> a4;

    fileRead["sigma_r"] >> sigma_r;
    fileRead["sigma_phi"] >> sigma_phi;

    fileRead["p_init_x_"] >> p_init_x_;
    fileRead["p_init_y_"] >> p_init_y_;
    fileRead["p_init_theta_"] >> p_init_theta_;

    fileRead["convar_x_"] >> convar_x_;
    fileRead["convar_y_"] >> convar_y_;
    fileRead["convar_theta_"] >> convar_theta_;

    fileRead["convar_measure0"] >> convar_measure0;
    fileRead["convar_measure1"] >> convar_measure1;
    fileRead["convar_measure2"] >> convar_measure2;
    fileRead["convar_measure3"] >> convar_measure3;
    fileRead["convar_measure4"] >> convar_measure4;


    fileRead["update_odom_linear_"] >> update_odom_linear_;
    fileRead["update_odom_angle_"] >> update_odom_angle_;
    fileRead["stamp_interval"] >> stamp_interval;

    fileRead["INIT_LOCALIZATION"] >> INIT_LOCALIZATION;
    fileRead["SELECT_MARK_FOR_INIT_"] >> SELECT_MARK_FOR_INIT_;
}
/**
 * @brief graphSlam::WorldToMark3
 * 将里程计坐标系 计算的robot(x,y,theta)三维量转化到以mark20为原点的坐标系.
 * @param dst
 * @param src
 */
void graphSlam::WorldToMark3(Point3f &dst, Point3f src)
{
    Point3f src_temp(src);
    dst.x =  cos(coordinate_angle_) * src_temp.x + sin(coordinate_angle_) * src_temp.y - coordinate_x_;
    dst.y = -sin(coordinate_angle_) * src_temp.x + cos(coordinate_angle_) * src_temp.y - coordinate_y_;
    dst.z =  src_temp.z - coordinate_angle_;
}

void graphSlam::WorldToMark3(Point3d &dst, Point3d src)
{
    Point3d src_temp(src);
    dst.x =  cos(coordinate_angle_) * src_temp.x + sin(coordinate_angle_) * src_temp.y - coordinate_x_;
    dst.y = -sin(coordinate_angle_) * src_temp.x + cos(coordinate_angle_) * src_temp.y - coordinate_y_;
    dst.z =  src_temp.z - coordinate_angle_;
}

void graphSlam::WorldToMark3(OdomMessage &dst, OdomMessage src)
{
    OdomMessage src_temp(src);
    dst.x =  cos(coordinate_angle_) * src_temp.x + sin(coordinate_angle_) * src_temp.y - coordinate_x_;
    dst.y = -sin(coordinate_angle_) * src_temp.x + cos(coordinate_angle_) * src_temp.y - coordinate_y_;
    dst.theta =  src_temp.theta - coordinate_angle_;
}

/**
 * @brief graphSlam::WorldToMark2
 * 将landmark2维状态量(x,y)转化到以mark20为原点的坐标系.
 * @param dst
 * @param src
 */
void graphSlam::WorldToMark2(Point3f &dst, Point3f src)
{
    Point3f src_temp(src);
    dst.x =  cos(coordinate_angle_) * src_temp.x + sin(coordinate_angle_) * src_temp.y - coordinate_x_;
    dst.y = -sin(coordinate_angle_) * src_temp.x + cos(coordinate_angle_) * src_temp.y - coordinate_y_;
}
void graphSlam::WorldToMark2(Point3d &dst, Point3d src)
{
    Point3d src_temp(src);
    dst.x =  cos(coordinate_angle_) * src_temp.x + sin(coordinate_angle_) * src_temp.y - coordinate_x_;
    dst.y = -sin(coordinate_angle_) * src_temp.x + cos(coordinate_angle_) * src_temp.y - coordinate_y_;
}


/**
 * @brief graphSlam::dataFilter
 * 滑动均值与冒泡排序剔除最大最小再进行均值。
 * @param v   速度缓冲表
 * @param w   角速度缓冲表
 */
void graphSlam::dataFilter(double &v, double  &w)
{
    Vodom[data_filter_num] = v;
    Wodom[data_filter_num]  = w;
    data_filter_num = (data_filter_num+1) % DATA_FUSION_NUM;
    double total_w = 0;
    double total_v = 0;
#if IS_OPEN_BUBBLE_FILTER
    bubbleSort(Vodom);
    bubbleSort(Wodom);
    for (int i=1; i<DATA_FUSION-1; ++i)
    {
        //fTest << " " <<  Vodom[i]  << "  " << Wodom[i] << "  " <<  DATA_FUSION <<  " "  << endl;
        total_v += Vodom[i];
        total_w += Wodom[i];
    }
    v = total_v / ( DATA_FUSION - 2);
    w = total_w / ( DATA_FUSION - 2);
#else
    for (int i=0; i<DATA_FUSION_NUM; ++i)
    {
        // fTest << " " <<  Vodom[i]  << "  " << Wodom[i] << "  " <<  DATA_FUSION <<  " "  << endl;
        total_v += Vodom[i];
        total_w += Wodom[i];
    }
    v = total_v / ( DATA_FUSION_NUM );
    w = total_w / ( DATA_FUSION_NUM );
#endif
    
}
void graphSlam::bubbleSort(double  unsorted[])
{
    for (int i = 0; i < DATA_FUSION_NUM; i++)
    {
        for (int j = i; j < DATA_FUSION_NUM; j++)
        {
            if (unsorted[i] > unsorted[j])
            {
                int temp = unsorted[i];
                unsorted[i] = unsorted[j];
                unsorted[j] = temp;
            }
        }
    }
}

/**
 * @brief graphSlam::vectorIsValid
 * 用于判断vector中是否存在有效landmark,
 *          如果存在，则返回true；
 *             否则，返回false.
 * @param vector_data
 * @return
 */
bool graphSlam::vectorIsValid(vector<CPointsFour> vector_data)
{
    for (int i=0; i<vector_data.size(); i++)
    {
        if (vector_data[i].ID!=-1)
            return true;
    }
    return false;
}
/*
 * Point3f graphSlam::diffCoordinate(CPointsFour mark_2d,CPointsFourWorld mark_2d_world)
 *    初始位置运算，
 * 输入：
 *     为均值化后的mark， 起始物理位置与世界位置
 * 输出：
 *     相对偏移值
 *
 */
Point3f graphSlam::diffCoordinate(CPointsFour mark_2d,CPointsFourWorld mark_2d_world)
{
    Point3f coordinate_data;
    float d_x ,d_y,d_theta ;
    d_x = 1.0* (    (mark_2d.corn0.X  - mark_2d_world.corn0.X) +(mark_2d.corn1.X - mark_2d_world.corn1.X)
                    +(mark_2d.corn2.X  - mark_2d_world.corn2.X) +(mark_2d.corn3.X - mark_2d_world.corn3.X)
                    +(mark_2d.center.X - mark_2d_world.center.X)
                    )/5 ;
    
    d_y = 1.0* (    (mark_2d.corn0.Y  - mark_2d_world.corn0.Y) +(mark_2d.corn1.Y - mark_2d_world.corn1.Y)
                    +(mark_2d.corn2.Y  - mark_2d_world.corn2.Y) +(mark_2d.corn3.Y - mark_2d_world.corn3.Y)
                    +(mark_2d.center.Y - mark_2d_world.center.Y)
                    )/5 ;
    //保留原mark 20 顺序的情况下
    //    d_theta = 1.0*(  atan2(  (mark_2d.corn3.Y - mark_2d.corn0.Y ),(mark_2d.corn3.X - mark_2d.corn0.X ))
    //                     +atan2( (mark_2d.corn2.Y - mark_2d.corn1.Y ),(mark_2d.corn2.X - mark_2d.corn1.X ))
    //                     )/2;
    d_theta = 1.0*(  atan2(  (mark_2d.corn3.Y - mark_2d.corn2.Y ),(mark_2d.corn3.X - mark_2d.corn2.X ))
                     +atan2( (mark_2d.corn0.Y - mark_2d.corn1.Y ),(mark_2d.corn0.X - mark_2d.corn1.X ))
                     )/2;
    coordinate_data.x = d_x;
    coordinate_data.y = d_y;
    coordinate_data.z = d_theta;
    return  coordinate_data;
}

/**
 * @brief graphSlam::storeSystemState
 * 系统状态量向量表保存
 * @param systemState  系统状态
 */
void graphSlam::storeSystemState( Mat systemState)
{
    Point3f robot_pose;
    robot_pose.x = systemState.at<float>(0);
    robot_pose.y = systemState.at<float>(1);
    robot_pose.z = systemState.at<float>(2);
    
    //#if IS_ALL_SYSTEM_AS_MARK_COORDNATE
    //    WorldToMark3(robot_pose,robot_pose);
    //#endif
    est_path_points_.push_back(robot_pose);
    
    Point3f landmark;
    vector<Point3f> landmarks;
    for (vector<int>::size_type t = 0; t < observed_landmark_num.size(); t++)
    {
        landmark.x = miu_state_.at<float>(3+t*2);
        landmark.y = miu_state_.at<float>(4+t*2);
        landmark.z = Lm_observed_Num[t];
        
        //#if IS_ALL_SYSTEM_AS_MARK_COORDNATE
        //        WorldToMark2(landmark,landmark);
        //#endif
        if (t >= landmarks_system_.size())  //绘制第一次出现的landmark的 ID
        {
            landmarks.push_back(landmark) ;
            landmarks_system_.push_back(landmarks) ;
        }
        else
        {
            landmarks_system_[t].push_back(landmark);
        }
    }
}

/**
 * @brief graphSlam::addObservationPos
 * 添加landmark 点（x,y） 到观测量中
 * @param landmarktemp     2d mark角点
 * @param id               2d mark ID
 * @return                 点集（x,y,id）  id = ID*5 + j (j=01234)
 */
Point3ffi graphSlam::addObservationPos(ConerPoint landmarktemp ,int id )
{
    Point2f delta;
    float dst;   //特征距离
    float theta; //特征角w.r.t.robot frame
    Point3ffi mark_temp;
    delta = Point2f(landmarktemp.X,landmarktemp.Y); //-Point2f(RobotPos.X,RobotPos.Y); //已经是delta值了
    //    dst = norm(delta);
    dst = sqrt( delta.x * delta.x + delta.y * delta.y) ;
    theta = atan2(delta.y, delta.x); //- robot_info_.Theta ;//    ??? the true path of vehicle
    angleWrap(theta);
    mark_temp.x = dst;
    mark_temp.y = theta;
    mark_temp.z = id; //temp value
    fobsevation <<" "<<  mark_temp.z <<" "<< mark_temp.x <<" "<< mark_temp.y <<endl;
    return mark_temp;
}

/**
 * @brief graphSlam::genGaussianValue
 * 高斯扰动量生成
 * @param Sigma2
 * @return
 */
float graphSlam::genGaussianValue(float Sigma2)
{///Generate a Gaussian value with variance sigma
    return rng.gaussian(sqrt(Sigma2));
}
/**
 * @brief graphSlam::displayMatrix
 * Mat 矩阵打印输出
 * @param matrix
 */
void graphSlam::displayMatrix(Mat matrix)
{
    for (int ii=0;ii<matrix.rows;ii++)
    {
        for (int jj=0;jj<matrix.cols;jj++)
        {
            std::cout << "  " << matrix.at<float>(ii,jj);
        }
        std::cout << "" << std::endl;
    }
}

void graphSlam::writeMatrix(Mat matrix ,ofstream& f_test)
{
    for (int ii=0;ii<matrix.rows;ii++)
    {
        for (int jj=0;jj<matrix.cols;jj++)
        {
            f_test << "  " << matrix.at<float>(ii,jj);
        }
        f_test << "" << std::endl;
    }
    f_test << " -------------------------------------  " << std::endl;
}

/**
 * @brief graphSlam::angleWrap
 * 角度归一化，规范到（-pi,pi）范围内
 * @param angle
 */
void graphSlam::angleWrap(float& angle)
{
    ///这个函数用来实现把角度规划到-pi至pi之间
    if (angle>=CV_PI)
        while (angle >= CV_PI)
        { angle=angle-2*CV_PI;}
    else if (angle<-1.0*CV_PI)
        while (angle < -1.0*CV_PI)
        { angle = angle+2*CV_PI;}
}
/**
 * @brief graphSlam::int2str
 * int 转换成 string
 * @param num
 * @return
 */
std::string  graphSlam::int2str(int num)
{
    std::stringstream ss;
    ss  <<  num;
    std::string text = ss.str();
    return text;
}

/**
 * @brief showRobotOrientation
 *   用于显示机器人的方向信息。这主要显示机器人的位置信息(odom与预测的具体值),同时以直线旋转的角度反应角度信息
 *   原本参考系是在odom,转换到map映射的坐标系下.
 * @param image
 * @param robot_info
 * @param rgb
 */
void graphSlam::showRobotOrientation(Mat image, OdomMessage rob_odom,Scalar rgb,int x_coordinate,int y_coordinate)
{
    const int ROBOT_DEFAULT_RADIUS = 2;
    const int ROBOT_DEFAULT_ARROW_LEN = 30;
    
    Point3d robot_pose;
    robot_pose.x = rob_odom.x;
    robot_pose.y = rob_odom.y;
    robot_pose.z = rob_odom.theta;
    //    WorldToMark3(robot_pose,robot_pose);       //采用地图固定下  初始坐标系调整就可以

#if  IS_ALL_SYSTEM_AS_MARK_COORDNATE
    WorldToMark3(robot_pose,robot_pose);       //初始坐标系调整就可以
#endif
    
    Point start, end;
    start.x = x_coordinate;
    start.y = y_coordinate;
    
    int thickness = 1;
    int lineType = 8;
    line( image,start,start+Point(500,0),CV_RGB(0,255,0),1,lineType );  //  x轴
    line( image,start,start+Point(0,500),CV_RGB(0,155,0),1,lineType );  //  y轴
    
    circle(image,start,ROBOT_DEFAULT_RADIUS,rgb,2,lineType );
    end.x = start.x + ROBOT_DEFAULT_ARROW_LEN * cos(robot_pose.z);  //放大5倍
    end.y = start.y - ROBOT_DEFAULT_ARROW_LEN * sin(robot_pose.z);  //display  y  convert
    line( image,start,end,rgb,thickness,lineType );
    
    //  标记坐标信息
    std::string text_id ="x: "+ float2str( robot_pose.x  );
    cv::putText(cv_camera_,text_id,Point(50,150),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    std::string text1 ="y: "+ float2str( robot_pose.y );
    cv::putText(cv_camera_,text1,Point(50,200),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    std::string text2 ="z: "+ float2str( robot_pose.z*180/ 3.14159 );
    cv::putText(cv_camera_,text2,Point(50,250),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    
    std::string text3 ="v: "+ float2str( rob_odom.v );
    cv::putText(cv_camera_,text3,Point(50,300),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    std::string text4 ="w: "+ float2str( rob_odom.w );
    cv::putText(cv_camera_,text4,Point(50,350),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
    
    
    
}

/**
 * @brief graphSlam::showRobotOrientation
 *        显示的是Ekfslam结果的系统状态    机器人信息
 * @param image
 * @param robot_info
 * @param rgb
 * @param x_coordinate
 * @param y_coordinate
 */
void graphSlam::showRobotOrientation(Mat image, Mat rob_update_mat,Scalar rgb,int x_coordinate,int y_coordinate)
{
    const int ROBOT_DEFAULT_RADIUS = 2;
    const int ROBOT_DEFAULT_ARROW_LEN = 30;
    
    Point3d robot_pose;
    robot_pose.x = rob_update_mat.at<float>(0);
    robot_pose.y = rob_update_mat.at<float>(1);
    robot_pose.z = rob_update_mat.at<float>(2);
    
    //#if IS_ALL_SYSTEM_AS_MARK_COORDNATE
    //    WorldToMark3(robot_pose,robot_pose);  //  miu_state 初始时已经转换了,无需再转换
    //#endif
    
    Point start, end;
    start.x = x_coordinate;
    start.y = y_coordinate;
    
    int thickness = 1;
    int lineType = 8;
    line( image,start,start+Point(500,0),CV_RGB(0,255,0),1,lineType );  //  x轴
    line( image,start,start+Point(0,500),CV_RGB(0,155,0),1,lineType );  //  y轴
    
    circle(image,start,ROBOT_DEFAULT_RADIUS,rgb,2,lineType );
    end.x = start.x + ROBOT_DEFAULT_ARROW_LEN * cos( robot_pose.z );
    end.y = start.y - ROBOT_DEFAULT_ARROW_LEN * sin( robot_pose.z );  //display  y  convert
    line( image,start,end,rgb,thickness,lineType );
    
    //  标记坐标信息
    std::string text_id ="x: "+ float2str( robot_pose.x  );
    cv::putText(cv_camera_,text_id,Point(300,150),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(255,0,0));
    std::string text1 ="y: "+ float2str( robot_pose.y );
    cv::putText(cv_camera_,text1,Point(300,200),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(255,0,0));
    std::string text2 ="z: "+ float2str( robot_pose.z *180/ 3.14159);
    cv::putText(cv_camera_,text2,Point(300,250),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(255,0,0));
    
    // 更新次数显示
    std::string text_predict_num ="predict: "+ std::to_string(num_EKFSlam_predict_);
    cv::putText(cv_camera_,text_predict_num,Point(300,300),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(255,0,0));
    std::string text_update_num ="update: "+ std::to_string(num_EKFSlam_update_);
    cv::putText(cv_camera_,text_update_num,Point(300,350),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(255,0,0));
    
    std::string text_time_interval_num ="interval: "+ std::to_string(num_time_interval);
    cv::putText(cv_camera_,text_time_interval_num,Point(300,400),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(255,0,0));
    
}

/**
 * @brief graphSlam::showRobot
 * 在mat 图片上绘制机器人位置
 * @param map          图片
 * @param robot_info   机器人信息（x,y,theta）
 * @param rgb          绘制时颜色选择
 */
void graphSlam::showRobot(cv::Mat& map, OdomMessage rob_odom,Scalar rgb)
{
    Point3d robot_pose;
    robot_pose.x = rob_odom.x;
    robot_pose.y = rob_odom.y;
    robot_pose.z = rob_odom.theta;
    //    WorldToMark3(robot_pose,robot_pose);       //采用地图固定下  初始坐标系调整就可以

#if IS_ALL_SYSTEM_AS_MARK_COORDNATE
    WorldToMark3(robot_pose,robot_pose);       //初始坐标系调整就可以
#endif
    
    const int ROBOT_DEFAULT_RADIUS = 2;
    const int ROBOT_DEFAULT_ARROW_LEN = 10;
    
    Point start, end;
    start.x = robot_pose.x + MAP_BASE_X_;
    start.y = robot_pose.y + MAP_BASE_Y_;
    
    start.y = map.rows - start.y;
    
    int thickness = 1;
    int lineType = 8;
    
    // int radius = 10;
    circle(map,start,ROBOT_DEFAULT_RADIUS,rgb,0,lineType );
    
    end.x = start.x + ROBOT_DEFAULT_ARROW_LEN * cos(robot_pose.z);
    //    end.y = start.y + ROBOT_DEFAULT_ARROW_LEN * sin(robot_info.Theta);  //display  y  convert ..
    end.y = start.y - ROBOT_DEFAULT_ARROW_LEN * sin(robot_pose.z);  //display  y  convert ..
    
    line( map,start,end,rgb,thickness,lineType );
}
/**
 * @brief graphSlam::showRobotTriangle
 * 在mat 图片上绘制机器人位置，以三角形式表示        -->单独做到一个截框界面显示 每步机器人信息（x,y,theta）
 * @param map          图片
 * @param robot_info   机器人信息（x,y,theta）
 * @param rgb          绘制时颜色选择
 */
void graphSlam::showRobotTriangle(cv::Mat& map, OdomMessage rob_odom, Scalar rgb)
{
    float width = 3;
    Point robot_pos;
    robot_pos.x =  rob_odom.x + MAP_BASE_X_ ;
    //     robot_pos.y = -robot_info.Y + MAP_BASE_Y_ ;    //odom robot 方向
    robot_pos.y = map.rows - (rob_odom.y + MAP_BASE_Y_);
    
    Point Pa,Pb,Pc,Pd,Pe;
    float width_cos = width*cos(rob_odom.theta);
    float width_sin = width*sin(rob_odom.theta);
    
    Pa.x = robot_pos.x + 1.7*width_cos; Pa.y = robot_pos.y + 1.7*width_sin;
    Pd.x = robot_pos.x - 1.7*width_cos; Pd.y = robot_pos.y - 1.7*width_sin;
    
    Pb.x = Pd.x + 2*width_sin; Pb.y = Pd.y + 2*width_cos;
    Pc.x = Pd.x - 2*width_cos; Pc.y = Pd.y - 2*width_sin;
    
    Pe.x = robot_pos.x + 3.7*width_cos; Pe.y = robot_pos.y - 3.7*width_sin;
    
    int thickness = 1;
    int lineType = 8;
    
    // int radius = 10;
    line( map,Pa,Pe,rgb,thickness,lineType );
    line( map,Pa,Pb,rgb,thickness,lineType );
    line( map,Pa,Pc,rgb,thickness,lineType );
    line( map,Pb,Pc,rgb,thickness,lineType );
}
/**
 * @brief graphSlam::showLandmark
 * 在map上绘制landmark的位置信息。 这是最后一帧式,不带保存历史信息.
 * @param map
 * @param rgb
 */
void graphSlam::showSystemStateLandmark(cv::Mat& map, Scalar rgb)
{
    // cv::Mat map_copy ;
    for (int t = 0; t < observed_landmark_num.size(); t++)
    {
        //#if DISPLAY_UNDER_MARK_COORDINATE
#if       IS_ALL_SYSTEM_AS_MARK_COORDNATE
        Point3d robot_pose_src, robot_pose_dst;
        robot_pose_src.x = miu_state.at<float>(3+t*2);
        robot_pose_src.y = miu_state.at<float>(4+t*2);     // ****坐标系反转  逆转实际为负  测量为正
        // robot_pose_src.z = miu_state.at<float>(2);
        WorldToMark3(robot_pose_dst,robot_pose_src);
        
        float X= robot_pose_dst.x + MAP_BASE_X_;
        float Y= robot_pose_dst.y + MAP_BASE_Y_;
#else
        float X= miu_state_.at<float>(3+t*2)+ MAP_BASE_X_;
        float Y= miu_state_.at<float>(4+t*2)+ MAP_BASE_Y_;
#endif
        //        float X= miu_state.at<float>(3+t*2)+ MAP_BASE_X_;
        //        float Y= miu_state.at<float>(4+t*2)+ MAP_BASE_Y_;
        Y = map.rows - Y ;
        state_miu << "  " << t << " " << X << "  " << Y;
        cv::circle(map,Point( X,Y),1,rgb,2); //绘制mark位置
        
#if  IS_ONE_POINT_AS_LANDMARK
        std::string text = int2str( observed_landmark_num.at(t) );
        //        if (t >= landmark_have_drawed)  //绘制第一次出现的landmark的 ID
        //        {
        //            cv::putText(map,text,Point(X,Y+20),CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0,0) );
        //        }
        cv::putText(map,text,Point(X,Y+20), CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0,0) );  //不带历史显示
        
        
#else
        std::string text = int2str(observed_landmark_num.at(t)/5);
        if (t%5 == 4)
        {
            cv::putText(map,text,Point(X,Y+20),CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0,0) );  //不带历史显示
        }
#endif
        
    }
    std::string  num_text = int2str(observed_landmark_num.size());
    cv::putText(cv_camera_,num_text,Point( 40,40),CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(0, 0,255) );
    state_miu << " " << endl;
    landmark_have_drawed = observed_landmark_num.size() ;
}

/**
 * @brief graphSlam::showLandmark
 * 在map上绘制landmark的位置信息。
 * @param map
 * @param rgb
 */
void graphSlam::showSystemStateRobot(cv::Mat& map, Scalar rgb)
{
    //#if DISPLAY_UNDER_MARK_COORDINATE
#if       IS_ALL_SYSTEM_AS_MARK_COORDNATE
    Point3d robot_pose_src, robot_pose_dst;
    robot_pose_src.x = miu_state.at<float>(0);
    robot_pose_src.y = miu_state.at<float>(1);     // ****坐标系反转  逆转实际为负  测量为正
    robot_pose_src.z = miu_state.at<float>(2);
    WorldToMark3(robot_pose_dst,robot_pose_src);   //转换成显示部分
    cout  <<  miu_state.cols  <<  " "  <<  miu_state.rows  <<  endl;
    frobot  <<  " "   <<  miu_state.at<float>(0)  << " " <<  miu_state.at<float>(1)  <<  endl;
    int temp_X = robot_pose_dst.x + MAP_BASE_X_;
    int temp_Y = robot_pose_dst.y + MAP_BASE_Y_;
    temp_Y = map.rows - temp_Y ;
    
    cv::circle(map,Point( temp_X,temp_Y),1,CV_RGB(0, 255,0),1); //绘制 robot
    
#else
    frobot  <<  " "   <<  miu_state_.at<float>(0)  << " " <<  miu_state_.at<float>(1)  <<  endl;
    int temp_X = miu_state_.at<float>(0) + MAP_BASE_X_;
    int temp_Y = miu_state_.at<float>(1) + MAP_BASE_Y_;
    temp_Y = map.rows - temp_Y ;
    cv::circle(map,Point( temp_X,temp_Y),1,CV_RGB(0, 255,0),1); //绘制 robot
#endif
    
    
}

void graphSlam::drawCoordinate(cv::Mat& mat)
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
/**
 * @brief graphSlam::storeData
 * 保存系统状态量到文件  ofstream fmiu("miu.txt")  中
 */
void graphSlam::storeData(void )
{
    fmiu << "     "<< robot_odom_.x << "     "<< robot_odom_.y << "     " << robot_odom_.theta << "     "  ;
    for (int i=0;i<miu_state_.rows;++i)
    {
        fmiu << miu_state_.at<float>(i) << "     " ;
    }
    fmiu << endl;
}
/**
 * @brief graphSlam::float2str
 * 数据类型转换 float 转换到 string
 * @param num
 * @return
 */
std::string  graphSlam::float2str(float num)
{
    std::stringstream ss;
    ss << num;
    std::string text = ss.str();
    return text;
}

//--------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------

void graphSlam::combineCallback(const nav_msgs::Odometry::ConstPtr& pOdom, const sensor_msgs::ImageConstPtr& pImg)  //回调中包含多个消息
{
    //    call_back++;
    //TODO
    odom_stamp_  = pOdom->header.stamp;
    image_stamp_ = pImg->header.stamp;
    fStampAll<<odom_stamp_<<"    "<<image_stamp_<<"  "<<endl;
    //getOdomData(pOdom);
    cout<<"start get sensor message!"<<endl;
    getOdomDataByAcc(pOdom); //odom 已经转换到地图坐标系下
    cout<<"get odom data!"<<endl;

    is_img_update_ = getImgData(pImg);    // 像素值
    cout<<"get img data!"<<endl;
    cout << "sensor data stamp: " << robot_odom_.stamp<<endl;
    fOdom << "stamp x y theta v w " << robot_odom_.stamp<<" "<<robot_odom_.x << " "<< robot_odom_.y << " " << robot_odom_.theta
          << " " << robot_odom_.v << " " << robot_odom_.w << std::endl;
    pixDataToMetricData();
    //    static bool FINISH_INIT_ODOM_STATIC = false;
    //      FINISH_INIT_ODOM_STATIC = true;
    if(IS_GET_INIT_ODOM_COORDINATE_)
    {
        ekfslam(robot_odom_);
        //#if !IS_ONE_POINT_AS_LANDMARK
        //        if(IS_SYSTEM_TRANSFER_)
        //        {
        //            mapTransforDiff(miu_state);     //五点式才有方向调整
        //        }
        //#endif
        //landmark 与 map 点显示
        posesPublish();
        storeData();
        showImage();
        cout << "odom: x y theta " <<robot_odom_.x << " "<< robot_odom_.y << " " << robot_odom_.theta<<endl;
        cout <<"  ekf: x y theta "<<miu_state_.at<float>(0)<<" "<< miu_state_.at<float>(1) <<" "<<miu_state_.at<float>(2)<<endl;
    }
    else if(is_img_update_)
    {
        if(addInitVectorFull())
        {
            cout<<"go into coordinate transfer "<<endl;
            computerCoordinate();
            IS_GET_INIT_ODOM_COORDINATE_ = true;
            cout<<"finish coordinate transfer "<<endl;

        }
    }
    is_odom_update  = false ;
    is_img_update_  = false;
    // fdebug<<"    "<<call_back<<" "<<ekf_start<<" "<<ekf_end<<"   "<<endl;
}

void graphSlam::posesPublish()
{
//    ///×××××××××××××××××××××××××××××××
//    tf::Quaternion q;
//    q.setRPY(0, 0, 0.25);
//    odom_to_img_.sendTransform ( tf::StampedTransform( tf::Transform(q, tf::Vector3(0.2, 0.2, 0.5)),
//                                                       ros::Time::now(),"map", "my_odom")  );

//    odom_to_img_.sendTransform ( tf::StampedTransform( tf::Transform(q, tf::Vector3(0.2, 0.2, 0.5)),
//                                                       ros::Time::now(),"map", "odom")  );
//    cout<<"finish tf of odom and img "<<endl;
//    //static int i =0;
//    geometry_msgs::PoseStamped pose_miu_map;
//    pose_miu_map.header.stamp = ros::Time::now();
//    pose_miu_map.header.frame_id = "my_odom";

//    pose_miu_map.pose.position.x =  robot_odom_.x/100;
//    pose_miu_map.pose.position.y =  robot_odom_.y/100;
//    pose_miu_map.pose.position.z =  0.4;

//    //tf::Quaternion q;
//    //q.setRPY(0, 0,0);
//    pose_miu_map.pose.orientation.x = 0.0;
//    pose_miu_map.pose.orientation.y = 0.0;
//    pose_miu_map.pose.orientation.z = 0.0;
//    pose_miu_map.pose.orientation.w = 1.0;

//    pose_publisher_.publish(pose_miu_map);

    // 建立tf树转换  odom_to_img_ ， map 到 odom
    tf::Quaternion q;
    q.setRPY(0, 0, coordinate_angle_);
    odom_to_img_.sendTransform ( tf::StampedTransform( tf::Transform(q, tf::Vector3(coordinate_x_/100.0, coordinate_x_/100.0, 0.5)),
                                                      ros::Time::now(),"odom", "map")  );
    tf::Quaternion q2;
    q2.setRPY(0, 0, -coordinate_angle_);
    odom_to_img_.sendTransform ( tf::StampedTransform( tf::Transform(q2, tf::Vector3(-coordinate_x_/100.0, -coordinate_x_/100.0, -0.5)),
                                                       ros::Time::now(),"map", "my_odom")  );

    cout<<"finish tf of odom and img "<<endl;
//    tf::StampedTransform transform;
//    listener_.lookupTransform("map", "odom",ros::Time(0), transform);


    visualization_msgs::Marker points;
    points.header.frame_id = "my_odom";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.05;
    points.scale.y = 0.05;
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;


    for (int i=0;i<(miu_state_.rows-3)/2;++i)
    {
        geometry_msgs::Point pose_miu_map;
        pose_miu_map.x =  miu_state_.at<float>(3+2*i)/100.0;
        pose_miu_map.y =  miu_state_.at<float>(3+2*i+1)/100.0;
        pose_miu_map.z =  0.4;
        points.points.push_back(pose_miu_map);
    }
    marker_pub_.publish(points);


    //miu_robot
    nav_msgs::Odometry robot_odom;
    robot_odom.header.stamp = ros::Time::now();
    robot_odom.header.frame_id = "odom";
    //robot_odom.child_frame_id = "my_odom";
    robot_odom.pose.pose.position.x = miu_state_.at<float>(0)/100.0;
    robot_odom.pose.pose.position.y = miu_state_.at<float>(1)/100.0;
    robot_odom.pose.pose.position.z = 0;

    tf::Quaternion q_robot;
    q_robot.setRPY(0, 0,miu_state_.at<float>(2));
    robot_odom.pose.pose.orientation.x = q_robot.getX();
    robot_odom.pose.pose.orientation.y = q_robot.getY();
    robot_odom.pose.pose.orientation.z = q_robot.getZ();
    robot_odom.pose.pose.orientation.w = q_robot.getW();
    robot_pose_publisher_.publish(robot_odom);




//    geometry_msgs::PoseStamped pose_miu_robot;
//    pose_miu_robot.header.stamp = ros::Time::now();
//    pose_miu_robot.header.frame_id = "my_odom";

//    pose_miu_robot.pose.position.x =  miu_state.at<float>(0)/100;
//    pose_miu_robot.pose.position.y =  miu_state.at<float>(1)/100;
//    pose_miu_robot.pose.position.z =  0.2;

//    tf::Quaternion q2;
//    q.setRPY(0, 0, miu_state.at<float>(2));
//    pose_miu_robot.pose.orientation.x = q2.getX();
//    pose_miu_robot.pose.orientation.y = q2.getY();
//    pose_miu_robot.pose.orientation.z = q2.getZ();
//    pose_miu_robot.pose.orientation.w = q2.getW();

//    robot_pose_publisher_.publish(pose_miu_robot);

}


/**
 * @brief graphSlam::getOdomData
 * 得到里程数据,单位为(厘米,弧度);记录初始的时间戳参考值odom_init_stamp;
 * @param odomMsg
 */
void graphSlam::getOdomData(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    static bool init_flag_stamp = true;  //第一次记录参考时间戳
    if(init_flag_stamp)
    {
        odom_init_stamp = odomMsg->header.stamp;
        init_flag_stamp = false;
    }
    odom_stamp_ = odomMsg->header.stamp;
    geometry_msgs::Quaternion orientation = odomMsg->pose.pose.orientation;
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
    double yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);
    
    robot_odom_.stamp = odomMsg->header.stamp;
    robot_odom_.theta = yaw;
    robot_odom_.x = odomMsg->pose.pose.position.x*100;
    robot_odom_.y = odomMsg->pose.pose.position.y*100;
    robot_odom_.v = odomMsg->twist.twist.linear.x*100;
    robot_odom_.w = -1.0*odomMsg->twist.twist.angular.z;
    
    odom_messages_vector.push_back(robot_odom_);  // 里程计数据向量表
    is_odom_update = true;
}

/**
 * @brief graphSlam::getOdomDataByAcc
 * 利用速度积分的方式得到里程数据,里程数据通过坐标系变换转换到地图坐标系下位置。
 * @param odomMsg
 */
void graphSlam::getOdomDataByAcc(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    static bool init_flag_stamp = true;  //第一次记录参考时间戳
    double delta_stamp = (image_stamp_ - odom_stamp_old_).toSec();

    Point2f motion;
    motion.x =  odomMsg->twist.twist.linear.x*100;
    motion.y =  1.0*odomMsg->twist.twist.angular.z;
    odom_velocities_.push_back(motion);

    if(!init_flag_stamp)
    {
        Point2f VEL = odom_velocities_.at(odom_velocities_.size()-2);  //数组从0开始  又存入一值
        Point3f robot_pose(robot_odom_.x,robot_odom_.y,robot_odom_.theta);
        Point3f increase = odomModel(VEL,robot_pose,delta_stamp);

        robot_odom_.x     += increase.x;
        robot_odom_.y     += increase.y;
        robot_odom_.theta += increase.z;
        float theta = robot_odom_.theta;
        angleWrap(theta);
        robot_odom_.theta = theta;
    }
    else
    {
        odom_init_stamp = odomMsg->header.stamp;
        init_flag_stamp = false;
    }

    robot_odom_.stamp = odomMsg->header.stamp;
    robot_odom_.v =  odomMsg->twist.twist.linear.x*100;
    robot_odom_.w = 1.0*odomMsg->twist.twist.angular.z;
    //    if(IS_GET_INIT_ODOM_COORDINATE_)
    //    {
    //        WorldToMark3(robot_odom_,robot_odom_);
    //    }
    odom_messages_vector.push_back(robot_odom_);  // 里程计数据向量表
    is_odom_update = true;
    odom_stamp_old_ = odom_stamp_;

}

Point3f graphSlam::odomModel(Point2f motion, Point3f robotPose,float delta_time)
{
    Point2f VEL = odom_velocities_.at(odom_velocities_.size()-2);
    VEL.x = motion.x;
    VEL.y = motion.y;  //  取y  标准正向
    //    if ( VEL.x < 0.0006  && VEL.x >= 0)  VEL.x = 0.0;//v
    //    else if ( VEL.x > -0.0006 && VEL.x <  0)  VEL.x = 0.0;
    if ( VEL.y <  0.000001  && VEL.y >= 0)  VEL.y = 0.000001;//w
    else if ( VEL.y > -0.000001  && VEL.y <  0)  VEL.y = -0.000001;

    Point3f miu_increase; //算法第三行计算xPred_SLAM时的3*1矩阵
    float last_miu_theta = robotPose.z ;//上一个miu_SLAM的角度
    angleWrap(last_miu_theta);
    //speed mode motion increase   Wd 不能为 0
    miu_increase.x =  -VEL.x/VEL.y * sin(last_miu_theta) + VEL.x/VEL.y * sin(last_miu_theta + VEL.y * delta_time);
    miu_increase.y =   VEL.x/VEL.y * cos(last_miu_theta) - VEL.x/VEL.y * cos(last_miu_theta + VEL.y * delta_time);
    miu_increase.z =   VEL.y * delta_time;

    //    Point3f robot_new_pose;
    //    robot_new_pose.x = robotPose.x + miu_increase.x ;
    //    robot_new_pose.y = robotPose.y + miu_increase.y ;
    //    robot_new_pose.z = robotPose.z + miu_increase.z ;
    //    angleWrap(miu_increase.z);
    return miu_increase;
}

/**
 * @brief graphSlam::getImgData
 *  获取图像信息,当获取的图像有定义编码is_img_update_返回true,否则返回假.
 *  观测到的信息都存储在 landmark5_vector_  ;  信息为沿机器人方向相对中心的(x,y)像素值
 * @param imgMsg
 */
bool graphSlam::getImgData(const sensor_msgs::ImageConstPtr& imgMsg)
{
    image_stamp_ = imgMsg->header.stamp;
    cv_bridge::CvImagePtr  cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(imgMsg,sensor_msgs::image_encodings::BGR8);
    cv_ptr->image.copyTo(cv_camera_);

    landmark5_pix_vector_.clear();
    landmark5_pix_vector_ = pQrDetect_->detectRawLandmarks(cv_ptr->image,visual_mark_num_);
    if ( vectorIsValid(landmark5_pix_vector_))
    {
        savePixDatas(landmark5_pix_vector_);  //保存detect 数据
        return true;
    }
    else
        return false;

//      cv::Mat test_img = imread("./test2/before0.png");
////    cv::Mat test_img2 ;
////    cvtColor( test_img, test_img, CV_BGR2GRAY );
////    equalizeHist(test_img,test_img2);  // 直方图均衡化.
////    static int count = 1;
////    string str0 = "./test2/cut_bag2_" + int2str(count) + ".jpg";
////    imwrite(str0,cv_ptr->image);
////    count++;

////    bool result = pQrDetect_->codeTestProcess(test_img);
//    bool result = pQrDetect_->codeTestProcess(cv_ptr->image);
////    if(!result)
////    {
////        cout<<" no code detect in raw image  "<<endl;
////    }
////    else
////    {
////        cout<<" detected"<<endl;
////    }
}

/**
 * @brief graphSlam::addInitVectorFull
 * 静止采集INIT_LOCALIZATION组数据.进行位置的相对值计算,最后用于地图旋转
 * 一旦数组采集完成,返回true.
 * @return
 */
bool graphSlam::addInitVectorFull()
{
    if (  abs(robot_odom_.x)<0.001&& abs(robot_odom_.y)<0.001)
    {
        for (int i=0; i<landmark5_meter_vector_.size();++i )
        {
            CPointsFour mark_2d = landmark5_meter_vector_.at(i);
            if (mark_2d.ID == SELECT_MARK_FOR_INIT_ )
            {
                mark5_init_vector_.push_back(mark_2d);
                break;
            }
        }
    }
    if ( INIT_LOCALIZATION == mark5_init_vector_.size() )
        return true;
    else
        return false;
}
/**
 * @brief graphSlam::pixDataToMetricData
 * 将像素值表示的点信息转化为米制表示,并存储起来.
 */
void graphSlam::pixDataToMetricData()
{
    landmark5_meter_vector_.clear();
    landmark5_meter_vector_side_.clear();
    for(vector<CPointsFour>::size_type i=0; i<landmark5_pix_vector_.size(); i++ )
    {
        CPointsFour  dst;
        CPointsFour  dst_side;
        CPointsFour  src = landmark5_pix_vector_.at(i);
        pQrDetect_->imgCornerToWorld( dst, src);         // 利用已知高度信息   //TODO
        pQrDetect_->pixToMeter( dst_side, src);         //  利用已知边长信息
        landmark5_meter_vector_.push_back(dst);
        landmark5_meter_vector_side_.push_back(dst_side);
    }
    saveMeterDatas(landmark5_meter_vector_,fQrMeterData);
    saveMeterDatas(landmark5_meter_vector_side_,fQrMeterDataSide);
}

void graphSlam::computerCoordinate()
{
    CPointsFour mark_2d = pQrDetect_->averageCPointsFour(mark5_init_vector_,10.0,0.0,2.0,6420);

    mark_2d_world_ = pQrDetect_->getInitMarkMessage(SELECT_MARK_FOR_INIT_);
    
    Point3f diff_data = diffCoordinate(mark_2d, mark_2d_world_);
    coordinate_x_ = diff_data.x ;
    coordinate_y_ = diff_data.y ;
    coordinate_angle_ = diff_data.z;
    fcoordinate_init<<"x y theta: "<<coordinate_x_<<"  "<<coordinate_y_<<"  "<<coordinate_angle_<<" "<<endl;
}

void graphSlam::savePixDatas( vector<CPointsFour> vector_data )
{
    fQrPixData  <<  vector_data.size();
    for (vector<CPointsFour>::size_type i=0; i< vector_data.size();i++)
    {
        fQrPixData  << " " <<  vector_data[i].ID  << " " <<  vector_data[i].corn0.X  << " " << vector_data[i].corn0.Y   << " " <<  vector_data[i].corn1.X << " " << vector_data[i].corn1.Y
                    << " " <<  vector_data[i].corn2.X  << " " << vector_data[i].corn2.Y   << " " <<  vector_data[i].corn3.X << " " << vector_data[i].corn3.Y
                    << " " <<  vector_data[i].center.X << " " << vector_data[i].center.Y;
    }
    fQrPixData  << " " <<  endl;
}

void graphSlam::saveMeterDatas( vector<CPointsFour> vector_data,ofstream&  writeData )
{
    writeData  <<  vector_data.size();
    for (vector<CPointsFour>::size_type i=0; i< vector_data.size();i++)
    {
        writeData  << " " <<  vector_data[i].ID  << " " <<  vector_data[i].corn0.X  << " " << vector_data[i].corn0.Y   << " " <<  vector_data[i].corn1.X << " " << vector_data[i].corn1.Y
                   << " " <<  vector_data[i].corn2.X  << " " << vector_data[i].corn2.Y   << " " <<  vector_data[i].corn3.X << " " << vector_data[i].corn3.Y
                   << " " <<  vector_data[i].center.X << " " << vector_data[i].center.Y;
    }
    writeData  << " " <<  endl;
}

void graphSlam::getObservations()
{
    // QrLandMark landmarktemp;
    Point3ffi   mark_temp;
    CPointsFour landmark5_temp;
    observations_.clear();  //清空观测量。只保存当前时刻的观测量
    
    for (vector<CPointsFour>::size_type i=0; i < landmark5_meter_vector_.size(); i++)
    {
        landmark5_temp = landmark5_meter_vector_.at(i);  //观测实际值
        //        if ( SELECT_MARK_FOR_INIT_ !=landmark5_temp.ID )  //起始点作为地图固定
        //        {
#if  IS_ONE_POINT_AS_LANDMARK  //1
        mark_temp = addObservationPos(landmark5_temp.center,landmark5_temp.ID) ;
        observations_.push_back(mark_temp); //landmark的index存储在Point3f的第三维中
#else//五点式
        mark_temp = addObservationPos(landmark5_temp.corn0, 5*landmark5_temp.ID + 0) ;
        observations_.push_back(mark_temp); //landmark的index存储在Point3f的第三维中
        mark_temp = addObservationPos(landmark5_temp.corn1, 5*landmark5_temp.ID + 1) ;
        observations_.push_back(mark_temp); //landmark的index存储在Point3f的第三维中
        mark_temp = addObservationPos(landmark5_temp.corn2, 5*landmark5_temp.ID + 2) ;
        observations_.push_back(mark_temp); //landmark的index存储在Point3f的第三维中
        mark_temp = addObservationPos(landmark5_temp.corn3, 5*landmark5_temp.ID + 3) ;
        observations_.push_back(mark_temp); //landmark的index存储在Point3f的第三维中
        mark_temp = addObservationPos(landmark5_temp.center, 5*landmark5_temp.ID + 4) ;
        observations_.push_back(mark_temp); //landmark的index存储在Point3f的第三维中
#endif
        //        }
    }
}

/**
 * @brief graphSlam::getNumQrcode
 * observations_        一帧图像管测量
 * observed_landmark_num   所有观测到的landmark id 向量表
 *
 * 逐步筛选observations_中的量，与已有的observed_landmark_num向量表匹配，
 * observations_中第一次观测到的角点加入observed_landmark_num向量表中
 */
int graphSlam::getLandmarkNum(void)
{
    bool IS_EVER_SEEN = false;
    for(vector<Point3ffi>::size_type i=0; i<observations_.size(); i++)
    {
        int Qid = observations_.at(i).z;
        for(vector<int>::size_type c=0; c<observed_landmark_num.size(); c++)
        {
            if(Qid == observed_landmark_num[c])//出现过
            {
                IS_EVER_SEEN = true;
            }
        }
        if(!IS_EVER_SEEN) //从未出现过
        {
            observed_landmark_num.push_back(Qid);
        }
        IS_EVER_SEEN = false;
    }
    return observed_landmark_num.size();
}
/**
 * @brief graphSlam::stateExtended
 * 状态miu_SLAM 扩维 一个码两个量（x,y）加入系统状态   系统状态从(num_old,1)扩展到(num,1);协方差从num_old方阵扩到num方阵
 * @param num
 * @param num_old
 * @return
 */
void graphSlam::stateExtended(int num, int num_old)
{
    //    这种扩维是正确的,但是与程序的miu_state先后有关
    //    for (int i = num_old; i< 3 + 2*num; i++)
    //    {
    //        miu_state.push_back(0.0);
    //    }
    //    cv::Mat tran = Mat::eye(num,num_old,CV_32FC1);

    //    miu_convar_p = tran.t() * miu_convar_p * tran;
    //    for (int i= 3 + 2*num_old; i < 3 + 2*num;i++)
    //    {
    //        miu_convar_p.at<float>(i,i) = 1000000;   //对角方差要大1000000
    //    }
    //    /* 矩阵扩维
    cv::Mat miu_state_new = Mat::zeros(3+2*num,1,CV_32FC1);   //已去掉跟s有关的项，
    for (int i = 0; i< 3 + 2*num_old; i++)
    {
        miu_state_new.at<float>(i) = miu_state_.at<float>(i);
    }
    miu_state_ = Mat::zeros(3 + 2*num,1,CV_32FC1);
    for (int i = 0; i<3 + 2*num_old; i++)
    {
        miu_state_.at<float>(i) = miu_state_new.at<float>(i);
    }
    
    cv::Mat miu_convar_p_new =  Mat::zeros(3 + 2*num, 3 + 2*num, CV_32FC1);  //这个可以放到初始化函数中去  ？？初值???
    
    for (int i = 0; i < 3 + 2*num_old; i++)
    {
        for (int j = 0;j < 3 + 2*num_old; j++)
        {
            miu_convar_p_new.at<float>(i,j) = miu_convar_p.at<float>(i,j);
        }
    }
    
    miu_convar_p = Mat::zeros(3 + 2*num, 3 + 2*num,CV_32FC1);
    for (int i = 0; i < 3 + 2*num_old; i++)
    {
        for (int j = 0; j < 3 + 2*num_old; j++)
            miu_convar_p.at<float>(i,j) = miu_convar_p_new.at<float>(i,j);
    }
    for (int i= 3 + 2*num_old; i < 3 + 2*num;i++)
    {
        miu_convar_p.at<float>(i,i) = 1000000;   //对角方差要大1000000
    }
    //    */
}
/**
 * @brief graphSlam::motionModel
 * 将系统运动模型更新,运动预测时系统状态不变.
 * @param motion               输入线速度与角速度(v,w)
 * @param system_state         系统状态量
 * @param system_state_convar  系统协方差量
 * @param variable_num         系统状态量数量
 * @param delta_time           时间间隔
 */
Point2f graphSlam::motionModel(Point2f motion, Mat& system_state, Mat& system_state_convar, int variable_num, float delta_time)
{
    Point2f VEL;
    VEL.x = motion.x;
    VEL.y = motion.y;  //  取y  标准正向
    if ( VEL.x < 0.0006  && VEL.x >= 0)  VEL.x = 0.0;//v
    else if ( VEL.x > -0.0006 && VEL.x <  0)  VEL.x = 0.0;
    if ( VEL.y <  0.000001  && VEL.y >= 0)  VEL.y = 0.000001;//w
    else if ( VEL.y > -0.000001  && VEL.y <  0)  VEL.y = -0.000001;
    cout << "Vd: " << VEL.x << "   " << " Wd " << VEL.y << " vd/wd " << VEL.x/VEL.y << endl;
    
    Mat Fx = Mat::zeros(3, 3 + 2*variable_num, CV_32FC1);
    Fx.at<float>(0,0) = 1.0;
    Fx.at<float>(1,1) = 1.0;
    Fx.at<float>(2,2) = 1.0;
    Mat miu_increase = Mat::zeros(3, 1, CV_32FC1); //算法第三行计算xPred_SLAM时的3*1矩阵

    float last_miu_theta = miu_state_.at<float>(2) ;//上一个miu_SLAM的角度
    angleWrap(last_miu_theta);
    //speed mode motion increase   Wd 不能为 0
    miu_increase.at<float>(0) =  -VEL.x/VEL.y * sin(last_miu_theta) + VEL.x/VEL.y * sin(last_miu_theta + VEL.y * delta_time);
    miu_increase.at<float>(1) =   VEL.x/VEL.y * cos(last_miu_theta) - VEL.x/VEL.y * cos(last_miu_theta + VEL.y * delta_time);
    miu_increase.at<float>(2) =   VEL.y * delta_time;
    Point2f robot_increase;
    robot_increase.x = sqrt( miu_increase.at<float>(0)* miu_increase.at<float>(0) + miu_increase.at<float>(1)*miu_increase.at<float>(1));
    robot_increase.y =  miu_increase.at<float>(2);

    //  prediction mean
    system_state = system_state + Fx.t()*miu_increase; // X'= X +Jaci_f(x)*delt(x)   predicted mean
    angleWrap(system_state.at<float>(2));
    
    //计算Gt   Jacibi_x(x,y,theita)
    Mat Gt = Mat::zeros(3 + 2*variable_num, 3+2*variable_num, CV_32FC1);
    Mat Gt_increase = Mat::zeros(3, 3, CV_32FC1); //用来计算Gt的3*3矩阵
    Mat I_SLAM = Mat::eye(3 + 2*variable_num, 3+2*variable_num, CV_32FC1); //算法中的单位矩阵
    Gt_increase.at<float>(0,2) = -VEL.x/VEL.y * cos(last_miu_theta) + VEL.x/VEL.y * cos(last_miu_theta+VEL.y * delta_time);
    Gt_increase.at<float>(1,2) = -VEL.x/VEL.y * sin(last_miu_theta) + VEL.x/VEL.y * sin(last_miu_theta+VEL.y * delta_time);
    Gt = I_SLAM + Fx.t() * Gt_increase*Fx ;
    
    Mat Vt = Mat::zeros(3,2,CV_32FC1);
    //计算Vt   Jacibi_u(v,w)
    Vt.at<float>(0,0) = (-sin(last_miu_theta) + sin(last_miu_theta+VEL.y * delta_time))/VEL.y;
    Vt.at<float>(0,1) = VEL.x*(sin(last_miu_theta)-sin(last_miu_theta+VEL.y * delta_time))/VEL.y/VEL.y+VEL.x * cos(last_miu_theta+VEL.y * delta_time)*delta_time/VEL.y;
    Vt.at<float>(1,0) = (cos(last_miu_theta) - cos(last_miu_theta+VEL.y * delta_time))/VEL.y;
    Vt.at<float>(1,1) = -VEL.x*(cos(last_miu_theta)-cos(last_miu_theta+VEL.y * delta_time))/VEL.y/VEL.y+VEL.x * sin(last_miu_theta+VEL.y * delta_time)*delta_time/VEL.y;
    Vt.at<float>(2,0) = 0;
    Vt.at<float>(2,1) = delta_time;
    
    Mat Mt = Mat::zeros(2,2,CV_32FC1);
    //计算Mt   motion noise ;  why add the motion noise   ?????
    Mt.at<float>(0,0) = a1*VEL.x*VEL.x + a2*VEL.y*VEL.y;
    Mt.at<float>(1,1) = a3*VEL.x*VEL.x + a4*VEL.y*VEL.y;
    //        //        Mt.at<float>(0,0) = a1;
    //        //        Mt.at<float>(0,1) = a2;
    //        //        Mt.at<float>(1,0) = a3;
    //        //        Mt.at<float>(1,1) = a4;
    
    Mat Rt = Mat::zeros(3,3,CV_32FC1); //vv
    Rt = Vt * Mt * Vt.t();//计算Rt
    //        Rt.at<float>(0,0) = convar_x_;
    //        Rt.at<float>(1,1) = convar_y_;
    //        Rt.at<float>(2,2) = convar_theta_;
    
    //  predict convar :system_state_convar
    cout << "----------------------------------------" << endl;
    
    system_state_convar = Gt * system_state_convar * Gt.t() + Fx.t() * Rt * Fx; //计算预测方差 Px
    return robot_increase;
}

/**
 * @brief graphSlam::updateSystemState
 * 系统状态根据观测进行更新;
 * @param system_state
 * @param system_state_convar
 * @param variable_num
 * @param observation_Marks_per_image
 */
void graphSlam::updateSystemState_with_odomObserve(Mat& system_state, Mat& system_state_convar, int variable_num, vector<Point3ffi>& observation_Marks_per_image)
{
    Mat Qt = Mat::zeros(5, 5, CV_32FC1);
    Qt.at<float>(0,0) = convar_measure0;
    Qt.at<float>(1,1) = convar_measure1;
    Qt.at<float>(2,2) = convar_measure2;
    Qt.at<float>(3,3) = convar_measure3;
    Qt.at<float>(4,4) = convar_measure4;

    Mat I_SLAM = Mat::eye(3+2*variable_num, 3+2*variable_num, CV_32FC1); //算法中的单位矩阵
    Mat miu_temp_sum = Mat::zeros(3+2*variable_num, 1, CV_32FC1); //用来计算Ht的2*5矩阵
    Mat Kt_i_Ht_sum = Mat::zeros(3+2*variable_num, 3+2*variable_num, CV_32FC1);
    
    Mat St = Mat::zeros(5, 5, CV_32FC1); //vv
    /////----------------------------update------------------------------------------------------------------
    int   j = -1;  //  the order of Qrmark  表示观察到的 特征标记      与状态量中landmark有关
    int   Qid = -1; //  the Id of Qrmark
    bool  IS_FIRST_OBSERVED;// = true;
    float dst;
    float theta;
    float distance_2;
    Point2f delta;
    Point2f z,zp;
    Mat delta_z;
    for (vector<Point3ffi>::size_type i=0; i<observation_Marks_per_image.size(); i++)
    {
        IS_FIRST_OBSERVED = true;
        Qid = observation_Marks_per_image.at(i).z;  //mark的ID
        z = Point2f(observation_Marks_per_image.at(i).x,observations_.at(i).y); //观测值  是极坐标
        for (vector<int>::size_type c=0; c<Lm_observed_Num.size(); ++c)
        {
            if (Qid == Lm_observed_Num[c])   //   出现过   逐一比对已经观察到的mark列表
            {
                IS_FIRST_OBSERVED = false;
                j = c;                     //  选取第j个landmark导入观察模型并进行状态更新。
            }
        }

        if (IS_FIRST_OBSERVED) //从未出现过
        {
            //           landmark_observed.at<float>(i)=j ;//保存住这次观测到的路标号
            landmarkObservation landmark_measure;
            landmark_measure.measure = z;
            landmark_measure.id = Qid;
            landmark_measure.predict = z;
            landmark_measure.stamp = time_bag_;
            landmark_measure.robot_odom = Point3f(robot_odom_.x,robot_odom_.y,robot_odom_.theta);
            landmark_measure.robot_predict = Point3f(system_state.at<float>(0),system_state.at<float>(1),system_state.at<float>(2));
            vector<landmarkObservation> temp;
            temp.push_back(landmark_measure);
            landmark_vectors_.push_back(temp);


            Lm_observed_Num.push_back(Qid) ;
            j = Lm_observed_Num.size()-1 ;                // 注意 数组从0开始  -1 ？？？？？？？？？？

            if( Qid == SELECT_MARK_FOR_INIT_ )
            {
                Init_mark_in_systemState_j = j;
            }
            // 在观测函数 x y 极坐标系下的值。
            system_state.at<float>(2*j+3) = system_state.at<float>(0) + z.x * cos( z.y + system_state.at<float>(2) );  //第j个landmark的y坐标
            system_state.at<float>(2*j+4) = system_state.at<float>(1) + z.x * sin( z.y + system_state.at<float>(2) );  //第j个landmark的x坐标
            
            fnewlandmark<<"robot x,y,theta "<<robot_odom_.x<<" "<<robot_odom_.y<<" " <<robot_odom_.theta<<"  "<<"j "<<j<<" Qid "<< Qid
                       <<" miu(01) "<<system_state.at<float>(0)<<" "<<system_state.at<float>(1)<<" "<<system_state.at<float>(2)<<
                         " 2*j+3  "<<system_state.at<float>(2*j+3) <<" 2*j+4 "<<system_state.at<float>(2*j+4)<<" z.x-y "<<z.x<<" "<<z.y<<endl;
        }
        else
        {
            if( SELECT_MARK_FOR_INIT_ == Qid/5 )   // 出现过就可以开始进行转移变换
            {
                IS_SYSTEM_TRANSFER_ = true;
            }
            //这里的z相当于landmark的标号  （全局坐标下： 地图值- 预测机器人值）与 观测值 z ： 表示mark与robot的相对距离
            delta = Point2f(system_state.at<float>(2*j+3), system_state.at<float>(2*j+4)) - Point2f(system_state.at<float>(0),system_state.at<float>(1));
            distance_2 = delta.x * delta.x + delta.y * delta.y ;
            dst = sqrt(distance_2);
            theta = atan2(delta.y, delta.x) - system_state.at<float>(2);      //偏离robot的方向角方向的角度   相对xy坐标系下值
            zp.x = dst;
            angleWrap(theta);
            zp.y = theta;

            landmarkObservation landmark_measure;
            landmark_measure.measure = z;
            landmark_measure.id = Qid;
            landmark_measure.predict = zp;
            landmark_measure.stamp = time_bag_;
            landmark_measure.robot_odom = Point3f(robot_odom_.x,robot_odom_.y,robot_odom_.theta);
            landmark_measure.robot_predict = Point3f(system_state.at<float>(0),system_state.at<float>(1),system_state.at<float>(2));
            landmark_vectors_[j].push_back(landmark_measure);

            //计算Fj
            Mat Fj = Mat::zeros(5, 3+2 * variable_num, CV_32FC1);
            Fj.at<float>(0,0) = 1;
            Fj.at<float>(1,1) = 1;
            Fj.at<float>(2,2) = 1;
            Fj.at<float>(3,2 * j+3) = 1;
            Fj.at<float>(4,2 * j+4) = 1;
            
            //计算Htjaccibi
            Mat Ht_temp = Mat::zeros(5, 5, CV_32FC1); //用来计算Ht的2*5矩阵

            Ht_temp.at<float>(0,0) = 1;
            Ht_temp.at<float>(1,1) = 1;
            Ht_temp.at<float>(2,2) = 1;

            Ht_temp.at<float>(3,0) = -delta.x * dst;
            Ht_temp.at<float>(3,1) = -delta.y * dst;
            Ht_temp.at<float>(3,3) =  delta.x * dst;
            Ht_temp.at<float>(3,4) =  delta.y * dst;
            Ht_temp.at<float>(4,0) =  delta.y;
            Ht_temp.at<float>(4,1) = -delta.x;
            Ht_temp.at<float>(4,2) = -distance_2;
            Ht_temp.at<float>(4,3) = -delta.y;
            Ht_temp.at<float>(4,4) =  delta.x;

            Mat Ht = Mat::zeros(5, 3 + 2*variable_num, CV_32FC1);
            //~~~~~~~~~~~~~~~~~~
            Ht=(1/distance_2) * Ht_temp * Fj ;
            St = Ht * system_state_convar * Ht.t()+Qt;
            
            Mat Kt = Mat::zeros(3 + 2*variable_num, 2, CV_32FC1); //
            Kt = system_state_convar * Ht.t() * St.inv();

            K_vec.clear();
            for(int tt=0; tt< Kt.rows; tt++ )
            {
                Point2f k_temp(Kt.at<float>(tt,0),Kt.at<float>(tt,1));
                K_vec.push_back(k_temp);
            }
            Kt_vectors_.push_back(K_vec);

            Point3f delta_robot;
            delta_robot.x = system_state.at<float>(0) - robot_odom_.x;
            delta_robot.y = system_state.at<float>(1) - robot_odom_.y;
            delta_robot.z = system_state.at<float>(2) - robot_odom_.theta;

            z = z-zp;       //  更新的处理
            angleWrap(z.y);
            
            delta_z = Mat::zeros(5,1,CV_32FC1);
            delta_z.at<float>(0) = delta_robot.x;
            delta_z.at<float>(1) = delta_robot.y;
            delta_z.at<float>(2) = delta_robot.z;

            delta_z.at<float>(3) = z.x;
            delta_z.at<float>(4) = z.y;

            
            system_state  =  system_state + Kt * delta_z;  // xPred_SLAM 关于landmark为极坐标值
            angleWrap(system_state_convar.at<float>(2));
            system_state_convar =  (I_SLAM - Kt * Ht) * system_state_convar;
        }
    }
    
    //            miu_temp_sum = miu_temp_sum + Kt * delta_z ;
    //            Kt_i_Ht_sum = Kt_i_Ht_sum + Kt * Ht;
    //        }
    //    }
    //    system_state = system_state + miu_temp_sum ;  // xPred_SLAM 关于landmark为极坐标值
    //    angleWrap(system_state.at<float>(2));
    //    system_state_convar = (I_SLAM - Kt_i_Ht_sum) * system_state_convar;

    system_state_convar =  0.5*(system_state_convar + system_state_convar.t());
}



/**
 * @brief graphSlam::updateSystemState
 * 系统状态根据观测进行更新;
 * @param system_state
 * @param system_state_convar
 * @param variable_num
 * @param observation_Marks_per_image
 */
void graphSlam::updateSystemState(Mat& system_state, Mat& system_state_convar, int variable_num, vector<Point3ffi>& observation_Marks_per_image)
{
    Mat Qt = Mat::zeros(2, 2, CV_32FC1);
    Qt.at<float>(0,0) = convar_measure3;
    Qt.at<float>(1,1) = convar_measure4;

    Mat I_SLAM = Mat::eye(3+2*variable_num, 3+2*variable_num, CV_32FC1); //算法中的单位矩阵
    Mat miu_temp_sum = Mat::zeros(3+2*variable_num, 1, CV_32FC1); //用来计算Ht的2*5矩阵
    Mat Kt_i_Ht_sum = Mat::zeros(3+2*variable_num, 3+2*variable_num, CV_32FC1);

    Mat St = Mat::zeros(2, 2, CV_32FC1); //vv
    /////----------------------------update------------------------------------------------------------------
    int   j = -1;  //  the order of Qrmark  表示观察到的 特征标记      与状态量中landmark有关
    int   Qid = -1; //  the Id of Qrmark
    bool  IS_FIRST_OBSERVED;// = true;
    float dst;
    float theta;
    float distance_2;
    Point2f delta;
    Point2f z,zp;
    Mat delta_z;
    for (vector<Point3ffi>::size_type i=0; i<observation_Marks_per_image.size(); i++)
    {
        IS_FIRST_OBSERVED = true;
        Qid = observation_Marks_per_image.at(i).z;  //mark的ID
        z = Point2f(observation_Marks_per_image.at(i).x,observations_.at(i).y); //观测值  是极坐标
        for (vector<int>::size_type c=0; c<Lm_observed_Num.size(); ++c)
        {
            if (Qid == Lm_observed_Num[c])   //   出现过   逐一比对已经观察到的mark列表
            {
                IS_FIRST_OBSERVED = false;
                j = c;                     //  选取第j个landmark导入观察模型并进行状态更新。
            }
        }

        if (IS_FIRST_OBSERVED) //从未出现过
        {
            //           landmark_observed.at<float>(i)=j ;//保存住这次观测到的路标号
            landmarkObservation landmark_measure;
            landmark_measure.measure = z;
            landmark_measure.id = Qid;
            landmark_measure.predict = z;
            landmark_measure.stamp = time_bag_;
            landmark_measure.robot_odom = Point3f(robot_odom_.x,robot_odom_.y,robot_odom_.theta);
            landmark_measure.robot_predict = Point3f(system_state.at<float>(0),system_state.at<float>(1),system_state.at<float>(2));
            vector<landmarkObservation> temp;
            temp.push_back(landmark_measure);
            landmark_vectors_.push_back(temp);


            Lm_observed_Num.push_back(Qid) ;
            j = Lm_observed_Num.size()-1 ;                // 注意 数组从0开始  -1 ？？？？？？？？？？

            if( Qid == SELECT_MARK_FOR_INIT_ )
            {
                Init_mark_in_systemState_j = j;
            }
            // 在观测函数 x y 极坐标系下的值。
            system_state.at<float>(2*j+3) = system_state.at<float>(0) + z.x * cos( z.y + system_state.at<float>(2) );  //第j个landmark的y坐标
            system_state.at<float>(2*j+4) = system_state.at<float>(1) + z.x * sin( z.y + system_state.at<float>(2) );  //第j个landmark的x坐标

            fnewlandmark<<"robot x,y,theta "<<robot_odom_.x<<" "<<robot_odom_.y<<" " <<robot_odom_.theta<<"  "<<"j "<<j<<" Qid "<< Qid
                       <<" miu(01) "<<system_state.at<float>(0)<<" "<<system_state.at<float>(1)<<" "<<system_state.at<float>(2)<<
                         " 2*j+3  "<<system_state.at<float>(2*j+3) <<" 2*j+4 "<<system_state.at<float>(2*j+4)<<" z.x-y "<<z.x<<" "<<z.y<<endl;
        }
        else
        {
            if( SELECT_MARK_FOR_INIT_ == Qid/5 )   // 出现过就可以开始进行转移变换
            {
                IS_SYSTEM_TRANSFER_ = true;
            }
            //这里的z相当于landmark的标号  （全局坐标下： 地图值- 预测机器人值）与 观测值 z ： 表示mark与robot的相对距离
            delta = Point2f(system_state.at<float>(2*j+3), system_state.at<float>(2*j+4)) - Point2f(system_state.at<float>(0),system_state.at<float>(1));
            distance_2 = delta.x * delta.x + delta.y * delta.y ;
            dst = sqrt(distance_2);
            theta = atan2(delta.y, delta.x) - system_state.at<float>(2);      //偏离robot的方向角方向的角度   相对xy坐标系下值
            zp.x = dst;
            angleWrap(theta);
            zp.y = theta;

            landmarkObservation landmark_measure;
            landmark_measure.measure = z;
            landmark_measure.id = Qid;
            landmark_measure.predict = zp;
            landmark_measure.stamp = time_bag_;
            landmark_measure.robot_odom = Point3f(robot_odom_.x,robot_odom_.y,robot_odom_.theta);
            landmark_measure.robot_predict = Point3f(system_state.at<float>(0),system_state.at<float>(1),system_state.at<float>(2));
            landmark_vectors_[j].push_back(landmark_measure);

            //计算Fj
            Mat Fj = Mat::zeros(5, 3+2 * variable_num, CV_32FC1);
            Fj.at<float>(0,0) = 1;
            Fj.at<float>(1,1) = 1;
            Fj.at<float>(2,2) = 1;
            Fj.at<float>(3,2 * j+3) = 1;
            Fj.at<float>(4,2 * j+4) = 1;

            //计算Htjaccibi
            Mat Ht_temp = Mat::zeros(2, 5, CV_32FC1); //用来计算Ht的2*5矩阵
            Ht_temp.at<float>(0,0) = -delta.x * dst;
            Ht_temp.at<float>(0,1) = -delta.y * dst;
            Ht_temp.at<float>(0,3) =  delta.x * dst;
            Ht_temp.at<float>(0,4) =  delta.y * dst;
            Ht_temp.at<float>(1,0) =  delta.y;
            Ht_temp.at<float>(1,1) = -delta.x;
            Ht_temp.at<float>(1,2) = -distance_2;
            Ht_temp.at<float>(1,3) = -delta.y;
            Ht_temp.at<float>(1,4) =  delta.x;
            Mat Ht = Mat::zeros(2, 3 + 2*variable_num, CV_32FC1);
            //~~~~~~~~~~~~~~~~~~
            Ht=(1/distance_2) * Ht_temp * Fj ;
            St = Ht * system_state_convar * Ht.t()+Qt;

            Mat Kt = Mat::zeros(3 + 2*variable_num, 2, CV_32FC1); //
            Kt = system_state_convar * Ht.t() * St.inv();

            K_vec.clear();
            for(int tt=0; tt< Kt.rows; tt++ )
            {
                Point2f k_temp(Kt.at<float>(tt,0),Kt.at<float>(tt,1));
                K_vec.push_back(k_temp);
            }
            Kt_vectors_.push_back(K_vec);

            z = z-zp;       //  更新的处理
            angleWrap(z.y);

            delta_z = Mat::zeros(2,1,CV_32FC1);
            delta_z.at<float>(0) = z.x;
            delta_z.at<float>(1) = z.y;

            system_state  =  system_state + Kt * delta_z;  // xPred_SLAM 关于landmark为极坐标值
            angleWrap(system_state_convar.at<float>(2));
            system_state_convar =  (I_SLAM - Kt * Ht) * system_state_convar;
        }
    }

    //            miu_temp_sum = miu_temp_sum + Kt * delta_z ;
    //            Kt_i_Ht_sum = Kt_i_Ht_sum + Kt * Ht;
    //        }
    //    }
    //    system_state = system_state + miu_temp_sum ;  // xPred_SLAM 关于landmark为极坐标值
    //    angleWrap(system_state.at<float>(2));
    //    system_state_convar = (I_SLAM - Kt_i_Ht_sum) * system_state_convar;
    system_state_convar =  0.5*(system_state_convar + system_state_convar.t());

}





/**
 * @brief graphSlam::showImage
 * 过程显示
 * 显示机器人位置  landmark  坐标系
 */
void  graphSlam::showImage()
{
    showRobot(raw_global_map_, robot_odom_, CV_RGB(0,0,0)) ;

    showSystemStateRobot(raw_global_map_,CV_RGB(0,0,255));
    drawCoordinate(raw_global_map_);
#if IS_OPEN_DYNAMIC_MAP
    showSystemStateLandmark(raw_global_map_,CV_RGB(0,0,255));
    slam_img_cvt_->convertOnce(raw_global_map_);
#else
    //    cv::Mat global_map_temp;
    //    raw_global_map_.copyTo(global_map_temp);  // 深度复制
    raw_global_map_.copyTo( global_map_display_ );
    showSystemStateLandmark( global_map_display_, CV_RGB(0,0,255));
    slam_img_cvt_->convertOnce( global_map_display_);
#endif
    showRobotOrientation(cv_camera_, robot_odom_, CV_RGB(0,0,0),50,50);
    showRobotOrientation(cv_camera_, miu_state_, CV_RGB(0,0,0),300,50);
    raw_img_cvt_->convertOnce(cv_camera_);
}

void graphSlam::ekfslam(OdomMessage rob_odom)
{
    time_bag_ = rob_odom.stamp;
    velocities_.push_back(Point2f(rob_odom.v,rob_odom.w));
    if (IS_INIT_FINISH_)
    {
        //        delta_t_ = (time_bag_ - time_bag_old).toSec(); //  秒
        delta_t_ = (image_stamp_ - time_bag_old).toSec(); //  秒

        cout << " motion predict" << endl;
        Point2f robot_vel = velocities_.at(velocities_.size()-2);  //数组从0开始  又存入一值
        Point2f robot_increase = motionModel(robot_vel, miu_state_, miu_convar_p, observed_mark_num_old, delta_t_);
        //miu_state.at<float>(2) = rob_odom.theta; // 假定角度正确

        num_EKFSlam_predict_++;
        cout << "observation start !" << endl;
        
        static Point3f last_update_pose(-2.0,-2.0,-1.0);
        Point2f update_increase,increase_xy;
        increase_xy.x = miu_state_.at<float>(0) - last_update_pose.x;
        increase_xy.y = miu_state_.at<float>(1) - last_update_pose.y;
        update_increase.x = sqrt( increase_xy.x * increase_xy.x + increase_xy.y * increase_xy.y);
        update_increase.y = abs( last_update_pose.z - miu_state_.at<float>(2) );
        
        //double delta_stamp = (image_stamp - odom_stamp).toSec();
        if(is_img_update_ ) //&& ( abs(delta_stamp) <= stamp_interval) )
        {
            num_time_interval++;
            //  if( ( (update_increase.x >= update_odom_linear_ ) || (update_increase.y >= update_odom_angle_ ) ) )
            if (1)
            {
                cout << " get observation" << endl;
                getObservations();
                int observed_mark_num = getLandmarkNum();
                cout << "--observed_mark_num_old--" << observed_mark_num_old << "--observed_mark_num----" << observed_mark_num << endl;
                if (observed_mark_num > observed_mark_num_old )
                {
                    stateExtended(observed_mark_num, observed_mark_num_old);
                }
                cout << " system update" << endl;
                //updateSystemState_with_odomObserve(miu_state, miu_convar_p, observed_mark_num, observations_);
                updateSystemState(miu_state_, miu_convar_p, observed_mark_num, observations_);

                observed_mark_num_old = observed_mark_num;
                num_EKFSlam_update_++;
                // 当固定mark出现大的偏差时就会触发,对整体系统状态进行旋转与偏移校正.
                
                last_update_pose.x = miu_state_.at<float>(0);
                last_update_pose.y = miu_state_.at<float>(1);
                last_update_pose.z = miu_state_.at<float>(2);
            }
        }
    }
    else
    {
        cout << " init system state and convarce !" << endl;
        time_bag_ = rob_odom.stamp;// odom_stamp;
        miu_state_ = Mat::zeros(3,1,CV_32FC1);       //已去掉跟s有关的项，原来是3+3*
        
        miu_state_.at<float>(0) = rob_odom.x;
        miu_state_.at<float>(1) = rob_odom.y;
        miu_state_.at<float>(2) = rob_odom.theta;
        
        miu_convar_p =  Mat::zeros(3,3,CV_32FC1);    //这个可以放到初始化函数中去  ？？初值
        miu_convar_p.at<float>(0,0) = p_init_x_ ; // 0.1;//0.1;//1;//100;//0.1;
        miu_convar_p.at<float>(1,1) = p_init_y_ ; //0.10;//0.1;//1;//100;//0.1;
        miu_convar_p.at<float>(2,2) = p_init_theta_ ; //0.38;//0.1;//0.38;
        
        IS_INIT_FINISH_ = true;
        //odom_theta = rob_odom.theta;
    }
    odom_theta_old = odom_theta;
    time_bag_old = time_bag_;
    storeSystemState(miu_state_);
}


void graphSlam::mapTransforDiff(Mat& sys_state)
{
    CPointsFour mark_2d;
    for (vector<int>::size_type c=0; c<Lm_observed_Num.size(); ++c)
    {
        switch (Lm_observed_Num[c]%(20*5))   //   出现过   逐一比对已经观察到的mark列表
        {
        case 0: mark_2d.corn0.init(sys_state.at<float>(3+2*c),sys_state.at<float>(4+2*c));break;
        case 1: mark_2d.corn1.init(sys_state.at<float>(3+2*c),sys_state.at<float>(4+2*c));break;
        case 2: mark_2d.corn2.init(sys_state.at<float>(3+2*c),sys_state.at<float>(4+2*c));break;
        case 3: mark_2d.corn3.init(sys_state.at<float>(3+2*c),sys_state.at<float>(4+2*c));break;
        case 4: mark_2d.center.init(sys_state.at<float>(3+2*c),sys_state.at<float>(4+2*c));break;
        default:break;
        }
    }
    map_lock_ = diffCoordinate(mark_2d, mark_2d_world_);
    fcoordinate_init<<"x y theta: "<<map_lock_.x<<"  "<<map_lock_.y<<"  "<<map_lock_.z<<" "<<endl;
    systemTransfore(sys_state);   // 坐标系调整就可以
    //    WorldToMark3(robot_pose,robot_pose);       //初始坐标系调整就可以
}

void graphSlam::systemTransfore(Mat& sys_state)
{
    Point3d dst;   //robot state         //robot_odom_   是里程计算上传式(固定式旋转与平移)非增量计算式, landmark一次调整大,后面变化小.
    cout<<"robot_odom_. (0 1 2)"<<robot_odom_.x <<" "<<robot_odom_.y <<" "<<robot_odom_.theta <<endl;
    cout<<"sys_state. (0 1 2)"<<sys_state.at<float>(0) <<" "<<sys_state.at<float>(1) <<" "<<sys_state.at<float>(2) <<endl;
    dst.x =  cos(map_lock_.z) * sys_state.at<float>(0) + sin(map_lock_.z) * sys_state.at<float>(1) - map_lock_.x;
    dst.y = -sin(map_lock_.z) * sys_state.at<float>(0) + cos(map_lock_.z) * sys_state.at<float>(1) - map_lock_.y;
    dst.z =  sys_state.at<float>(2) - map_lock_.z;

    sys_state.at<float>(0) = dst.x;
    sys_state.at<float>(1) = dst.y;
    sys_state.at<float>(2) = dst.z;

    cout<<"sys_state. (0 1 2)"<<sys_state.at<float>(0) <<" "<<sys_state.at<float>(1) <<" "<<sys_state.at<float>(2) <<endl;
    //    cout<<"sys_state. (0 1 2)"<<dst.x <<" "<<dst.y <<" "<<dst.z<<endl;
    for(int i=0; i<Lm_observed_Num.size(); i++)
    {
        cout<<"sys_state. i(3/4+2*i)"<<i<<" "<<sys_state.at<float>(3+2*i) <<" "<<sys_state.at<float>(4+2*i) <<endl;
        dst.x =  cos(map_lock_.z) * sys_state.at<float>(3+2*i) + sin(map_lock_.z) * sys_state.at<float>(4+2*i) - map_lock_.x;
        dst.y = -sin(map_lock_.z) * sys_state.at<float>(3+2*i) + cos(map_lock_.z) * sys_state.at<float>(4+2*i) - map_lock_.y;
        sys_state.at<float>(3+2*i) = dst.x;
        sys_state.at<float>(4+2*i) = dst.y;
        cout<<"sys_state. i(3/4+2*i)"<<i<<" "<<sys_state.at<float>(3+2*i) <<" "<<sys_state.at<float>(4+2*i) <<endl;
        //        cout<<"sys_state. i(3/4+2*i)"<<i<<" "<<dst.x <<" "<<dst.y <<endl;
    }
}


//======================================================================
//========================================================================================

/**
 * @brief graphSlam::getImgData
 *  获取图像信息,当获取的图像有定义编码is_img_update_返回true,否则返回假.
 *  观测到的信息都存储在 landmark5_vector_  ;  信息为沿机器人方向相对中心的(x,y)像素值
 * @param imgMsg
 */
bool graphSlam::getImgDataNew(const sensor_msgs::ImageConstPtr& imgMsg)
{
    image_stamp_ = imgMsg->header.stamp;
    cv_bridge::CvImagePtr  cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(imgMsg,sensor_msgs::image_encodings::BGR8);
    cv_ptr->image.copyTo(cv_camera_);

    image_data_ = cv_ptr->image ;
    cout<<" i have get the image! "<<endl;
    return true;
}

void graphSlam::combineCallbackNew(const nav_msgs::Odometry::ConstPtr& pOdom, const sensor_msgs::ImageConstPtr& pImg)  //回调中包含多个消息
{
    //TODO
    odom_stamp_  = pOdom->header.stamp;
    image_stamp_ = pImg->header.stamp;
    fStampAll<<odom_stamp_<<"    "<<image_stamp_<<"  "<<endl;
    getOdomData(pOdom);
    cout<<"start get sensor message!"<<endl;
    //getOdomDataByAcc(pOdom); //odom 已经转换到地图坐标系下
    cout<<"get odom data!"<<endl;

    is_img_update_ = getImgDataNew(pImg);    // 像素值
    cout<<"get img data!"<<endl;
    fOdom << "stamp x y theta v w " << robot_odom_.stamp<<" "<<robot_odom_.x << " "<< robot_odom_.y << " " << robot_odom_.theta
          << " " << robot_odom_.v << " " << robot_odom_.w << std::endl;
    //pixDataToMetricData();
    ekfslamNew(robot_odom_);      //EKFSLAM
    //landmark 与 map 点显示
    posesPublish();
    storeData();
    showImage();
    cout << "odom: x y theta " <<robot_odom_.x << " "<< robot_odom_.y << " " << robot_odom_.theta<<endl;
    cout <<"  ekf: x y theta "<<miu_state_.at<float>(0)<<" "<< miu_state_.at<float>(1) <<" "<<miu_state_.at<float>(2)<<endl;
    is_odom_update  = false ;
    is_img_update_  = false;
}

void graphSlam::ekfslamNew(OdomMessage rob_odom)
{
    time_bag_ = rob_odom.stamp;
    velocities_.push_back(Point2f(rob_odom.v,rob_odom.w));
    if (IS_INIT_FINISH_)  // 系统状态已初始化
    {
        delta_t_ = (image_stamp_ - time_bag_old).toSec(); //  秒
        cout << " motion predict" << endl;
        Point2f robot_vel = velocities_.at(velocities_.size()-2);  //数组从0开始  又存入一值
        Point3f robot_increase = motionModelNew(robot_vel, miu_state_, miu_convar_p, observed_mark_num_old, delta_t_);
        //miu_state.at<float>(2) = rob_odom.theta; // 假定角度正确
        num_EKFSlam_predict_++;
        cout << "observation start !" << endl;
        Point2f increase_d_seita;
        increase_d_seita.x = sqrt(robot_increase.x * robot_increase.x + robot_increase.y * robot_increase.y);
        increase_d_seita.y = abs(robot_increase.z);

        if(is_img_update_ )
        {
            num_time_interval++;
            //if( ( (increase_d_seita.x >= 5 ) || (increase_d_seita.y >= 0.067) ) )  //5cm   4°
           // if (1)
            {
                if(imgProcess())   //提取到二维码图像处理
                {
                    pixDataToMetricData();
                    cout << " get img observation" << endl;
                }
                getObservations();
                int observed_mark_num = getLandmarkNum();
                cout << "--observed_mark_num_old--" << observed_mark_num_old << "--observed_mark_num----" << observed_mark_num << endl;
                if (observed_mark_num > observed_mark_num_old )
                {
                    stateExtended(observed_mark_num, observed_mark_num_old);
                }
                cout << " system update" << endl;
                updateSystemState(miu_state_, miu_convar_p, observed_mark_num, observations_);
                observed_mark_num_old = observed_mark_num;
                num_EKFSlam_update_++;
                // 当固定mark出现大的偏差时就会触发,对整体系统状态进行旋转与偏移校正. ???
            }
        }
    }
    else   //系统状态进行初始化
    {
        cout << " init system state and convarce !" << endl;
        time_bag_ = rob_odom.stamp;// odom_stamp;
        miu_state_ = Mat::zeros(3,1,CV_32FC1);       //已去掉跟s有关的项，原来是3+3*

        miu_state_.at<float>(0) = rob_odom.x;
        miu_state_.at<float>(1) = rob_odom.y;
        miu_state_.at<float>(2) = rob_odom.theta;

        miu_convar_p =  Mat::zeros(3,3,CV_32FC1);    //这个可以放到初始化函数中去  ？？初值
        miu_convar_p.at<float>(0,0) = p_init_x_ ; // 0.1;//0.1;//1;//100;//0.1;
        miu_convar_p.at<float>(1,1) = p_init_y_ ; //0.10;//0.1;//1;//100;//0.1;
        miu_convar_p.at<float>(2,2) = p_init_theta_ ; //0.38;//0.1;//0.38;

        IS_INIT_FINISH_ = true;
    }
    odom_theta_old = odom_theta;
    time_bag_old = time_bag_;
    storeSystemState(miu_state_);
}

/**
 * @brief graphSlam::motionModelNew
 * 将系统运动模型更新,运动预测时系统状态不变.
 * @param motion               输入线速度与角速度(v,w)
 * @param system_state         系统状态量
 * @param system_state_convar  系统协方差量
 * @param variable_num         系统状态量数量
 * @param delta_time           时间间隔
 */
Point3f graphSlam::motionModelNew(Point2f motion, Mat& system_state, Mat& system_state_convar, int variable_num, float delta_time)
{
    Point2f VEL;
    VEL.x = motion.x;
    VEL.y = motion.y;  //  取y  标准正向
    if ( VEL.x < 0.0006  && VEL.x >= 0)
        VEL.x = 0.0;//v
    else if ( VEL.x > -0.0006 && VEL.x <  0)
        VEL.x = 0.0;
    if ( VEL.y <  0.000001  && VEL.y >= 0)
        VEL.y = 0.000001;//w
    else if ( VEL.y > -0.000001  && VEL.y <  0)
        VEL.y = -0.000001;
    cout << "Vd: " << VEL.x << "   " << " Wd " << VEL.y << " vd/wd " << VEL.x/VEL.y << endl;

    Mat miu_increase = Mat::zeros(3, 1, CV_32FC1); //算法第三行计算xPred_SLAM时的3*1矩阵
    float last_miu_theta = miu_state_.at<float>(2) ;//上一个miu_SLAM的角度
    angleWrap(last_miu_theta);
    //speed mode motion increase   Wd 不能为 0
    miu_increase.at<float>(0) =  -VEL.x/VEL.y * sin(last_miu_theta) + VEL.x/VEL.y * sin(last_miu_theta + VEL.y * delta_time);
    miu_increase.at<float>(1) =   VEL.x/VEL.y * cos(last_miu_theta) - VEL.x/VEL.y * cos(last_miu_theta + VEL.y * delta_time);
    miu_increase.at<float>(2) =   VEL.y * delta_time;

    Point3f robot_increase;
    robot_increase.x = miu_increase.at<float>(0);
    robot_increase.y = miu_increase.at<float>(1);
    robot_increase.z = miu_increase.at<float>(2);

    //  prediction mean
    Mat Fx = Mat::zeros(3, 3 + 2*variable_num, CV_32FC1);
    Fx.at<float>(0,0) = 1.0;
    Fx.at<float>(1,1) = 1.0;
    Fx.at<float>(2,2) = 1.0;
    system_state = system_state + Fx.t()*miu_increase;  // X'= X +Jaci_f(x)*delt(x)   predicted mean
    angleWrap(system_state.at<float>(2));

    //计算Gt   Jacibi_x(x,y,theita)
    Mat Gt = Mat::zeros(3 + 2*variable_num, 3+2*variable_num, CV_32FC1);
    Mat Gt_increase = Mat::zeros(3, 3, CV_32FC1); //用来计算Gt的3*3矩阵
    Mat I_SLAM = Mat::eye(3 + 2*variable_num, 3+2*variable_num, CV_32FC1); //算法中的单位矩阵
    Gt_increase.at<float>(0,2) = -VEL.x/VEL.y * cos(last_miu_theta) + VEL.x/VEL.y * cos(last_miu_theta+VEL.y * delta_time);
    Gt_increase.at<float>(1,2) = -VEL.x/VEL.y * sin(last_miu_theta) + VEL.x/VEL.y * sin(last_miu_theta+VEL.y * delta_time);
    Gt = I_SLAM + Fx.t() * Gt_increase*Fx ;

    Mat Vt = Mat::zeros(3,2,CV_32FC1);
    //计算Vt   Jacibi_u(v,w)
    Vt.at<float>(0,0) = (-sin(last_miu_theta) + sin(last_miu_theta+VEL.y * delta_time))/VEL.y;
    Vt.at<float>(0,1) = VEL.x*(sin(last_miu_theta)-sin(last_miu_theta+VEL.y * delta_time))/VEL.y/VEL.y+VEL.x * cos(last_miu_theta+VEL.y * delta_time)*delta_time/VEL.y;
    Vt.at<float>(1,0) = (cos(last_miu_theta) - cos(last_miu_theta+VEL.y * delta_time))/VEL.y;
    Vt.at<float>(1,1) = -VEL.x*(cos(last_miu_theta)-cos(last_miu_theta+VEL.y * delta_time))/VEL.y/VEL.y+VEL.x * sin(last_miu_theta+VEL.y * delta_time)*delta_time/VEL.y;
    Vt.at<float>(2,0) = 0;
    Vt.at<float>(2,1) = delta_time;

    Mat Mt = Mat::zeros(2,2,CV_32FC1);
    Mt.at<float>(0,0) = a1*VEL.x*VEL.x + a2*VEL.y*VEL.y;
    Mt.at<float>(1,1) = a3*VEL.x*VEL.x + a4*VEL.y*VEL.y;

    Mat Rt = Mat::zeros(3,3,CV_32FC1); //vv
    Rt = Vt * Mt * Vt.t();//计算Rt

    //  predict convar :system_state_convar
    cout << "----------------------------------------" << endl;

    system_state_convar = Gt * system_state_convar * Gt.t() + Fx.t() * Rt * Fx; //计算预测方差 Px
    return robot_increase;
}

/**主动搜索设计
 * @brief graphSlam::imgProcess
 * 处理的数据为图像 image_data_
 * @return
 */
bool graphSlam::imgProcess()
{
    total_search_count_++;

    double global_time = -1,local_time = -1;
    struct timeval tim1;
    struct timeval tim2;
    gettimeofday(&tim1, NULL);
    gettimeofday(&tim2, NULL);

    if(FIRST_DETEDCTET_)
    {//TODO: 全局搜索
        cout<<" global search "<<endl;

        globalSearch(image_data_);
        FIRST_DETEDCTET_ = evaluate(best_mark_center_);
        cout<<" best_mark_center_"<<best_mark_center_.x<<" "<<best_mark_center_.y<<"  "<<best_mark_center_.z<<endl;
        global_search_count_++;

        gettimeofday(&tim2, NULL);
        global_time = (tim2.tv_sec - tim1.tv_sec)*1000000.0 + (tim2.tv_usec - tim1.tv_usec);//算法耗时说明 微秒
    }
    else
    {//TODO: 局部搜索
        FIRST_DETEDCTET_ = true;
        cout<<" local search "<<endl;
        //Point2f local_trace_mark_center;
        cv::Mat local_img = imgPredict(best_local_mark_center_);
        cout<<" id local_img "<<best_local_mark_center_.z<<"  "<<local_img.cols<<"  "<<local_img.rows<<"   "<<endl;
        cout<<" best_local_mark_center_ "<<best_local_mark_center_.x<<"  "<<best_local_mark_center_.y<<"   "<<endl;

        bool loclal_found = localSearch(local_img,best_local_mark_center_);
        if(loclal_found)
        {
            FIRST_DETEDCTET_ = evaluate(best_local_mark_center_);
            local_search_count_++;
        }
        gettimeofday(&tim2, NULL);
        local_time = (tim2.tv_sec - tim1.tv_sec)*1000000.0 + (tim2.tv_usec - tim1.tv_usec);//算法耗时说明 微秒
    }
    //total global local local_time   global_time
   fSearchCount<<"   "<<total_search_count_<<" "<<global_search_count_<<"  "<<local_search_count_
        <<"  "<<local_time<<"  "<<global_time<<endl;

}

/**  调用二维码信息提取程序
 * @brief graphSlam::imgDetect
 * @param image
 * @return
 */
bool graphSlam::imgDetect(cv::Mat image)
{
    landmark5_pix_vector_.clear();
//    landmark5_pix_vector_ = pQrDetect_->detectRawLandmarksNew(image,visual_mark_num_);
    landmark5_pix_vector_ = pQrDetect_->detectRawLandmarks(image,visual_mark_num_);
//    cv::Mat im2 = imread("./2dmark.png");
//   landmark5_pix_vector_ = pLocalQrDetect_->detectRawLandmarks(im2,visual_mark_num_);
    if ( vectorIsValid(landmark5_pix_vector_))
    {
        savePixDatas(landmark5_pix_vector_);  //保存detect 数据
        return true;
    }
    else
        return false;
}
/**  调用二维码信息提取程序
 * @brief graphSlam::imgDetect
 * @param image
 * @return
 */
bool graphSlam::imgDetect(cv::Mat img, Point3f pix_offset)
{
    cout<<" img "<<img.cols<<"  "<<img.rows<<"   "<<endl;
    landmark5_pix_vector_.clear();
//     imshow("test",img);
//     cv::waitKey(20);
//     landmark5_pix_vector_ = pQrDetect_->detectRawLandmarksNew(img,visual_mark_num_);
   // cv::Mat im2 = imread("./2dmark.png");
   landmark5_pix_vector_ = pLocalQrDetect_->detectRawLandmarks(img,visual_mark_num_);

    cout<<" landmark5_pix_vector_.size "<< landmark5_pix_vector_.size()<<"   "<<endl;

    for(int i=0; i<landmark5_pix_vector_.size(); i++)
    {
        landmark5_pix_vector_.at(i).addOffset(pix_offset.x,pix_offset.y);
    }
    if(landmark5_pix_vector_.size() == 0)   return false;
    if ( vectorIsValid(landmark5_pix_vector_))
    {
        savePixDatas(landmark5_pix_vector_);  //保存detect 数据
        return true;
    }
    else
        return false;
}

bool graphSlam::globalSearch(cv::Mat img)
{
    imgDetect(img);
}

bool graphSlam::localSearch(cv::Mat img, Point3f local_trace_mark_center)
{
    bool flag = imgDetect(img,local_trace_mark_center);
    return flag;
}
/**
 * @brief graphSlam::evaluate
 * @param local_center
 * @return
 */
bool graphSlam::evaluate(Point3f &best_mark_center)
{
    float min = 800.0;
    float Radius ;
    for(int i=0; i<landmark5_pix_vector_.size();i++)
    {
        float distance = sqrt( (landmark5_pix_vector_.at(i).center.X - 320) * (landmark5_pix_vector_.at(i).center.X - 320)
                               + (landmark5_pix_vector_.at(i).center.Y -240 ) *(landmark5_pix_vector_.at(i).center.Y -240 ));
        if( distance < min )
        {
            Radius = sqrt( (landmark5_pix_vector_.at(i).corn0.X - landmark5_pix_vector_.at(i).corn3.X) * (landmark5_pix_vector_.at(i).corn0.X - landmark5_pix_vector_.at(i).corn3.X)
                           + (landmark5_pix_vector_.at(i).corn0.Y - landmark5_pix_vector_.at(i).corn3.Y) * (landmark5_pix_vector_.at(i).corn0.Y - landmark5_pix_vector_.at(i).corn3.Y));
            min = distance;
            best_mark_center.x = landmark5_pix_vector_.at(i).center.X ;
            best_mark_center.y = landmark5_pix_vector_.at(i).center.Y ;
            best_mark_center.z = landmark5_pix_vector_.at(i).ID*5+4; //注意这选择的是五点法
        }
    }
    best_mark_center_id_ = best_mark_center.z;
    best_mark_center_side_ = Radius;
    float value = 1- min/400.0;
    float threshold_min = Radius/400.0 ;//距边缘 二维码直径两倍内;
    if(value < threshold_min)   //可信度低
    {
        best_mark_center.z = -1;
        return true;
    }
    else
    {
        return  false;
    }

}

/**
 * @brief graphSlam::imgPredict
 *    observe_increase_image   预测二维码中心点的像素位置,截取二维码
 * @return
 */
cv::Mat graphSlam::imgPredict(Point3f& local_trace_mark_center)
{
   Point3f pos_world ;
   Point2f landmark_world;
   Point3f observe_increase_world;   //(dx dy seita)
   Point2f observe_increase_camera,observe_increase_pix;

   pos_world.x = miu_state_.at<float>(0);
   pos_world.y = miu_state_.at<float>(1);
   pos_world.z = miu_state_.at<float>(2);
   cout<<" pos_world "<<pos_world.x<<" "<<pos_world.y<<"  "<<pos_world.z<<endl;

   int index_landmark = findLandmarkId(best_mark_center_id_);  // best_mark_center_.z 注意这选择的是五点法 id
   cout<<" "<<index_landmark<<endl;

   landmark_world.x = miu_state_.at<float>(3+2*index_landmark);
   landmark_world.y = miu_state_.at<float>(4+2*index_landmark);
   cout<<" landmark_world "<<landmark_world.x<<" "<<landmark_world.y<<endl;

   observe_increase_world.x = landmark_world.x - pos_world.x;
   observe_increase_world.y = landmark_world.y - pos_world.y;
   observe_increase_world.z = pos_world.z;
   cout<<" observe_increase_world "<<observe_increase_world.x<<" "<<observe_increase_world.y<<"  "<<observe_increase_world.z<<endl;

   observe_increase_camera = worldToCamera(observe_increase_world);
   cout<<" observe_increase_camera "<<observe_increase_camera.x<<" "<<observe_increase_camera.y<<endl;

   observe_increase_pix = cameraToImage(observe_increase_camera,index_landmark);
   cout<<" observe_increase_pix "<<observe_increase_pix.x<<" "<<observe_increase_pix.y<<endl;

   //local_trace_mark_center = observe_increase_pix;
//   int cutRectX = (observe_increase_pix.x -  2*best_mark_center_side_) < 0 ? 0 : observe_increase_pix.x -  best_mark_center_side_;
//   int cutRectY = (observe_increase_pix.y -  2*best_mark_center_side_) < 0 ? 0 : observe_increase_pix.y -  best_mark_center_side_;

   int cutRectX = (observe_increase_pix.x -  best_mark_center_side_) <  0 ? 0 : observe_increase_pix.x -  best_mark_center_side_  ;
   int cutRectY = (observe_increase_pix.y -  best_mark_center_side_) <  0 ? 0: observe_increase_pix.y -  best_mark_center_side_;

   cutRectX = (cutRectX + 2*best_mark_center_side_) > 640 - 1 ? 640 - best_mark_center_side_ * 2 : cutRectX;
   cutRectY = (cutRectY + 2*best_mark_center_side_) > 480 - 1 ? 480 - best_mark_center_side_ * 2 : cutRectY;

   int cutRectSizeX = best_mark_center_side_ * 2;
   int cutRectSizeY = best_mark_center_side_ * 2;
   cout<<" cutRectX "<<cutRectX<<" cutRectX "<<cutRectY<<endl;
   Rect rect(cutRectX, cutRectY, cutRectSizeX, cutRectSizeY);

   local_trace_mark_center.x = cutRectX;
   local_trace_mark_center.y = cutRectY;
   local_trace_mark_center.z = best_mark_center_id_;

   cv::Mat cut_img;
   image_data_(rect).copyTo(cut_img);
   cv::rectangle(image_data_,rect,CV_RGB(0,255,0),1,8);
   cout<<" draw 1 "<<endl;
   cv::circle(image_data_,cvPoint( local_trace_mark_center.x,local_trace_mark_center.y),
          10,CV_RGB(0,255,0),-1); //绘制mark位置

   cv::circle(image_data_,cvPoint( local_trace_mark_center.x+2*best_mark_center_side_,local_trace_mark_center.y+best_mark_center_side_),
          10,CV_RGB(0,0,255),-1); //绘制mark位置

   cout<<" draw 1 "<<endl;
   cut_img_->convertOnce( cut_img);
   local_img_->convertOnce(image_data_);

   cout<<" local show "<<endl;
   return cut_img;
}

int graphSlam::findLandmarkId(int id)
{
    // Lm_observed_Num  vector 里面元素值为id, 其排列顺序为,系统状态landmark的排列顺序.
  for (int j=0; j< Lm_observed_Num.size();j++ )
  {
     if(id == Lm_observed_Num.at(j))
        return j;
  }
   return -1;
}

Point2f graphSlam::worldToCamera(Point3f observe_increase_world)
{
    Point2f observe_increase_camera;
    observe_increase_camera.x = observe_increase_world.x*cos(observe_increase_world.z) -
                                observe_increase_world.y*sin(observe_increase_world.z) ;
    observe_increase_camera.y = observe_increase_world.x*sin(observe_increase_world.z) +
                                 observe_increase_world.y*cos(observe_increase_world.z) ;
    return observe_increase_camera;
}

Point2f graphSlam::cameraToImage(Point2f observe_increase_camera, int id)
{
   Point2f observe_increase_image  ;
   observe_increase_image = pQrDetect_->trueToimPos(observe_increase_camera,(best_mark_center_.z-4)/5);
   return observe_increase_image;

}



//#####################################
void graphSlam::graphSlamCallBack(const nav_msgs::Odometry::ConstPtr& pOdom, const sensor_msgs::ImageConstPtr& pImg)  //回调中包含多个消息
{
    //TODO
   // VertexSE2* robot =  new VertexSE2;
    odom_new_ = getOdomData(pOdom);
    cout<<"get odom data!"<<endl;
    if(IS_FIRST_POS_)
    {
        robot_index_new_ = total_mark_ + 1;
        VertexSE2* robot =  new VertexSE2();
        robot->setId(robot_index_new_);
        robot->setEstimate( Eigen::Isometry2d::Identity() );
        robot->setFixed(true);
        optimizer.addVertex(robot);
        IS_FIRST_POS_ = false;

        odom_old_ = odom_new_;
        robot_index_old_ = robot_index_new_;
    }
    else
    {
        odom_trans = estimateMotion(odom_new_, odom_old_);
        if(IsMotionBig(odom_trans, angle_min_, distance_min_))  //运动距离很大
        {
          cout<<" motion big !"<<endl;
          // 机器人点
          robot_index_new_ += 1;
          VertexSE2* robot =  new VertexSE2();
          robot->setId(robot_index_new_);
          robot->setEstimate( Eigen::Isometry2d::Identity() );

          optimizer.addVertex(robot);
          IS_FIRST_POS_ = false;

          // 里程边
          g2o::EdgeSE3* odom_edge = new g2o::EdgeSE2();
          odom_edge->vertices() [0] = optimizer.vertex( odom_old_ );
          odom_edge->vertices() [1] = optimizer.vertex( odom_new_ );



          // 里程边 信息矩阵
          Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
          // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
          // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
          // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
          information(0,0) = information(1,1) = information(2,2) = 100;
          information(3,3) = information(4,4) = information(5,5) = 100;
          // 也可以将角度设大一些，表示对角度的估计更加准确
          odom_edge->setInformation( information );

//##############################################################
          img_new_ = getImgDataNew(pImg);   //  robot_index_new_   作为图像id
          images_vector.push_back(img_new_);//图像序列
          code_marks =  codeDetect(img_new_);
          cout<<"get img data!"<<endl;
          if(IsLoopClosure(images_vector))  //检测到闭环
          {
            //加一条里程边

          }

          VertexSE2* landmark =  new VertexSE2();
          landmark->setId(robot_index_new_);
          landmark->setEstimate( Eigen::Isometry2d::Identity() );

          optimizer.addVertex(robot);
          IS_FIRST_POS_ = false;

          odom_old_ = odom_new_;
          robot_index_old_ = robot_index_new_;

          odom_old_ = odom_new_;
          robot_index_old_ = robot_index_new_;
        }

    }

    is_odom_update  = false ;
    is_img_update_  = false;
}














