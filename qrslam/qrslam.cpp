/* read me :
 *
*/
#include "qrslam.h"

ofstream fmiu("./OutPut/Fmiu.txt");
ofstream fmotionPredict("./OutPut/FmotionPredict.txt");
ofstream fQrPixData("./OutPut/Fqr_pix_data.txt");
ofstream fQrMeterData("./OutPut/Fqr_meter_data.txt");
ofstream fQrMeterDataSide("./OutPut/Fqr_meter_data_side.txt");
ofstream fcoordinate_init("./OutPut/Fcoordinate_init.txt");
ofstream fStampAll("./OutPut/Fstampal.txt");
ofstream fDebug("./OutPut/debug.txt");

CVSlam::CVSlam(char* addr,char* config):transport_( ar_handle)
{
    Is_Global_search_ = true;
    search_info_.init(0, 0, 0, 0, 0, 0.0, 0.0);  //主动搜索相关信息
    readConfigure(config);

    num_time_interval = 0;
    num_EKFSlam_predict_ = 0;
    num_EKFSlam_update_ = 0;
    Fixed_landmark_index_ = -1;

    is_odom_update_ = false;
    is_img_update_  = false;

    landmark_num_ = 0;

    coordinate_x_ = 0.0;
    coordinate_y_ = 0.0;
    coordinate_angle_ = 0.0;

    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    solver_ = new OptimizationAlgorithmGaussNewton(blockSolver);
    optimizer_.setAlgorithm(solver_);

    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(ar_handle, "/odom", 1);
    img_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(ar_handle, "/usb_cam/image_raw", 1);

    sync_ = new  message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(10), *odom_sub_, *img_sub_);
    sync_->registerCallback(boost::bind(&CVSlam::startSLAM,this, _1, _2));

    map_pose_publisher_ = ar_handle.advertise< geometry_msgs::PoseStamped >("map_pose", 1);
    robot_pose_publisher_ = ar_handle.advertise<nav_msgs::Odometry>("robot_pose", 1);
    marker_pub_ = ar_handle.advertise<visualization_msgs::Marker>("map_marker", 10);
    systemStates_ = ar_handle.advertise<cv_slam::systemState>("systemStates", 10);



    raw_img_cvt_   =  new ImageConverter("/compare_img");
    slam_img_cvt_  =  new ImageConverter("/slam_map");
    cut_img_  =  new ImageConverter("/cut_img");

    raw_global_map_ =  cv::imread("./OutPut/map/map.png", CV_LOAD_IMAGE_COLOR); //for display
    //raw_global_map_ = cv::Mat::ones(map_width_, map_height_, CV_32FC1)*255;  //imread("test.png",1);; //for display
    map_scale_ = 0.25;
    map_size_.width  = raw_global_map_.cols * map_scale_;
    map_size_.height = raw_global_map_.rows * map_scale_;

    Mat map_ = Mat(map_size_,CV_32FC1);

    if (! raw_global_map_.data )
    {
       cout<<"Could not open or find the image :raw_global_map_ ."<<endl;
    }
    else
    {
        cv::line(raw_global_map_,cv::Point(map_base_x_, map_base_y_),cv::Point(map_base_x_, map_height_),CV_RGB(255,255,0),1,8);
        cv::line(raw_global_map_,cv::Point(map_base_x_, map_base_y_),cv::Point(map_width_, map_base_y_),CV_RGB(255,255,0),1,8);
    }

    pQrDetect_ = new DetctQrcode(addr,Fixed_mark_ID_);
    robot_odom_.init(0.0, 0.0, 0.0, 0.0, 0.0 ,ros::Time::now());

    fQrPixData  << "size id c0x y c1x y c2x y c3x y center_x y " << endl;
    fQrMeterData  << "size id c0x y c1x y c2x y c3x y center_x y " << endl;
    fQrMeterDataSide  << "size id c0x y c1x y c2x y c3x y center_x y " << endl;
    fmiu << " robot_odom_.x  .y .theta  miu(robot:3) miu(landmark:3N)  x y theta    " << endl;

    odom_stamp_ = ros::Time::now();
    odom_stamp_old_ = ros::Time::now();
    last_rob_pose_ = Point3f(-20,-20,-1);
}

CVSlam::~CVSlam()
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

    string file_name = "./OutPut/map/" + float2str(month)+'-'+float2str(data)+"-"+float2str(hour)
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
    if(!solver_)
        delete solver_;
}

//========================= 配置信息 ==========================
void CVSlam::readConfigure(char* config)
{
    cv::FileStorage fileRead(config, cv::FileStorage::READ);

    fileRead["MAP_BASE_Y_"] >> map_base_x_;
    fileRead["MAP_BASE_X_"] >> map_base_y_;

    fileRead["a1"] >> a1_;
    fileRead["a2"] >> a2_;
    fileRead["a3"] >> a3_;
    fileRead["a4"] >> a4_;

    fileRead["sigma_r"] >> sigma_r_;
    fileRead["sigma_phi"] >> sigma_phi_;

    fileRead["p_init_x_"] >> p_init_x_;
    fileRead["p_init_y_"] >> p_init_y_;
    fileRead["p_init_theta_"] >> p_init_theta_;

    fileRead["convar_x_"] >> convar_x_;
    fileRead["convar_y_"] >> convar_y_;
    fileRead["convar_theta_"] >> convar_theta_;

    fileRead["convar_measure0"] >> convar_measure0_;
    fileRead["convar_measure1"] >> convar_measure1_;
    fileRead["convar_measure2"] >> convar_measure2_;
    fileRead["convar_measure3"] >> convar_measure3_;
    fileRead["convar_measure4"] >> convar_measure4_;


    fileRead["update_odom_linear_"] >> update_odom_linear_;
    fileRead["update_odom_angle_"] >> update_odom_angle_;
    fileRead["stamp_interval"] >> stamp_interval_;

    fileRead["INIT_LOCALIZATION"] >> INIT_LOCALIZATION_;
    fileRead["Fixed_mark_ID_"] >> Fixed_mark_ID_;
}
//============================== SLAM ===========================
void CVSlam::startSLAM(const nav_msgs::Odometry::ConstPtr& pOdom, const sensor_msgs::ImageConstPtr& pImg)  //回调中包含多个消息
{
    odom_stamp_  = pOdom->header.stamp;
    image_stamp_ = pImg->header.stamp;
    fStampAll<<odom_stamp_<<"    "<<image_stamp_<<"  "<<endl;

    cout<<"start get sensor message!"<<endl;
    //getOdomDataByAcc(pOdom); //odom 已经转换到地图坐标系下
    robot_odom_ = getOdomDataNew(pOdom);
    cout<<"get odom data!"<<endl;
    image_data_ = getImgDataNew(pImg);    //获取图像帧 image_data_
    cout<<"get img data!"<<endl;
    static bool IS_GET_INIT_ODOM_COORDINATE = false;
    if(IS_GET_INIT_ODOM_COORDINATE)   // 未初始化坐标系
    {
        robot_odom_ = odomToWorld(robot_odom_, init_frame_diff_);
        cout << "odom: x y theta " <<robot_odom_.x << " "<< robot_odom_.y << " " << robot_odom_.theta<<endl;
        //TODO:
        ekfslamNew(robot_odom_, image_data_ );      //EKFSLAM

       // posesPublishNew();//landmark 与 map 点显示
        saveSystemState(fmiu);
       // showImageNew();
        pubSystemStates();
        cout <<"  ekf: x y theta "<<miu_state_.at<float>(0)<<" "<< miu_state_.at<float>(1) <<" "<<miu_state_.at<float>(2)<<endl;
    }
    else
    {
        bool IS_FULL_INIT = addInitPos(image_data_,robot_odom_,Fixed_mark_ID_,INIT_LOCALIZATION_);
        if( IS_FULL_INIT)
        {
            cout<<"go into coordinate transfer "<<endl;
            mark_2d_world_ = pQrDetect_->getInitMarkMessage(Fixed_mark_ID_);

            init_frame_diff_ = initFrameDiff(mark5_init_vector_, mark_2d_world_);
            saveInitFrameDiff(init_frame_diff_, fcoordinate_init);

            coordinate_x_ = init_frame_diff_.x ;
            coordinate_y_ = init_frame_diff_.y ;
            coordinate_angle_ = init_frame_diff_.z;
            cout<<"x y theta: "<<coordinate_x_<<"  "<<coordinate_y_<<"  "<<coordinate_angle_<<" "<<endl;

            IS_GET_INIT_ODOM_COORDINATE = true;
            cout<<"finish coordinate transfer "<<endl;
        }
    }
    is_odom_update_  = false ;
    is_img_update_  = false;
}

/**
 * @brief QrSlam::stateExtended
 * 状态miu_SLAM 扩维 一个码两个量（x,y）加入系统状态   系统状态从(num_old,1)扩展到(num,1);协方差从num_old方阵扩到num方阵
 * @param num
 * @param num_old
 * @return
 */
void CVSlam::stateExtended(const int curr_num, const int num_old)
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
    cv::Mat miu_state_new = Mat::zeros(3+3*curr_num,1,CV_32FC1);   //已去掉跟s有关的项，
    for (int i = 0; i< 3 + 3*num_old; i++)
    {
        miu_state_new.at<float>(i) = miu_state_.at<float>(i);
    }
    miu_state_ = Mat::zeros(3 + 3*curr_num,1,CV_32FC1);
    for (int i = 0; i<3 + 3*num_old; i++)
    {
        miu_state_.at<float>(i) = miu_state_new.at<float>(i);
    }

    cv::Mat miu_convar_p_new =  Mat::zeros(3 + 3*curr_num, 3 + 3*curr_num, CV_32FC1);  //这个可以放到初始化函数中去  ？？初值???

    for (int i = 0; i < 3 + 3*num_old; i++)
    {
        for (int j = 0;j < 3 + 3*num_old; j++)
        {
            miu_convar_p_new.at<float>(i,j) = miu_convar_p_.at<float>(i,j);
        }
    }

    miu_convar_p_ = Mat::zeros(3 + 3*curr_num, 3 + 3*curr_num,CV_32FC1);
    for (int i = 0; i < 3 + 2*num_old; i++)
    {
        for (int j = 0; j < 3 + 3*num_old; j++)
            miu_convar_p_.at<float>(i,j) = miu_convar_p_new.at<float>(i,j);
    }
    for (int i= 3 + 3*num_old; i < 3 + 3*curr_num;i++)
    {
        miu_convar_p_.at<float>(i,i) = 1000000;   //对角方差要大1000000
    }
    //    */
}


void CVSlam::ekfslamNew(OdomMessage rob_odom, Mat& img)
{
    static bool IS_finished_EKF_init = false;
    static int index_node = 1;
    //time_bag_ = rob_odom.stamp;
    velocities_.push_back(Point2f(rob_odom.v,rob_odom.w));
    if (IS_finished_EKF_init)  // 系统状态已初始化
    {
        float time_diff = (rob_odom.stamp - time_bag_old).toSec(); //  秒
        cout << "delta_time: " <<time_diff<< endl;
        cout << " motion predict" << endl;
        Point2f robot_vel = velocities_.at(velocities_.size()-2);  //数组从0开始  又存入一值
//        fmotionPredict<<"  "<<robot_vel.x <<" "<<robot_vel.y<<" ";
//        fmotionPredict<<"  "<<rob_odom.x <<" "<<rob_odom.y<<" "<<" "<<rob_odom.theta<<" ";
//        fmotionPredict<<"  "<<miu_state_.at<float>(0)<<" "<<miu_state_.at<float>(1)<<" "<<miu_state_.at<float>(2);

        Point3f robot_increase = motionModelNew(robot_vel, miu_state_, miu_convar_p_, landmark_num_, time_diff);
//        fmotionPredict<<"  "<<miu_state_.at<float>(0)<<" "<<miu_state_.at<float>(1)<<" "<<miu_state_.at<float>(2)<<endl;

        //miu_state.at<float>(2) = rob_odom.theta; // 假定角度正确
        num_EKFSlam_predict_++;

        Point2f increase_d_seita;
        increase_d_seita.x = sqrt(robot_increase.x * robot_increase.x + robot_increase.y * robot_increase.y);
        increase_d_seita.y = abs(robot_increase.z);

       if( ( (increase_d_seita.x >= 10 ) || (increase_d_seita.y >= 0.067) ) )  //5cm   4°
     //  if (1)  //运动触发
        {
            cout << "observation start !" << endl;
            Is_Global_search_ = true;    //TODO:   屏蔽局部搜索
            vector<CPointsFour> marks_pix = imgProcessNew(img, Is_Global_search_, best_mark_center_, search_info_);
            if(!marks_pix.empty())       //  提取到二维码图像处理
            {
                saveMarksPerImgPix(marks_pix, fQrPixData);      //保存pix
                PicNode pic_node = getPicInfo(marks_pix);     //获取每个节点的图像信息;  pic_node

                vector<CPointsFour> marks_metric = pixToMetric(marks_pix);
                saveMarksPerImgMetric(marks_metric, fQrMeterData);
                //saveMeterDatas(landmark5_meter_vector_side_,fQrMeterDataSide);
                cout << " get img observation" << endl;

                //getObservationsNew();  // TODO
                current_marks_observed_ = getObservationsNew(marks_metric); //  current observe:  x.y.theta.id
                cout<<" observationsNew_.size: "<<current_marks_observed_.size()<<" "<<endl;
                for(int t=0; t<current_marks_observed_.size(); t++)
                {
                fDebug<<" "<<current_marks_observed_.at(t).id<<" "<<current_marks_observed_.at(t).x<<" "<<current_marks_observed_.at(t).y<<" "<<current_marks_observed_.at(t).theta;
                 }
                fDebug<<""<<endl;
                int curr_landmark_num = getLandmarkNumNew(current_marks_observed_, id_all_marks_);  //为了系统状态扩维使用
                cout << "--observed_mark_num_old--" << landmark_num_ << "--observed_mark_num----" << curr_landmark_num << endl;
                if (curr_landmark_num > landmark_num_ )
                {
                    stateExtended(curr_landmark_num, landmark_num_);
                }

                cout << " system update" << endl;
                updateSLAM(miu_state_, miu_convar_p_, curr_landmark_num, current_marks_observed_, index_landmarks_);

                // 坐标系的固定调整..
                cout << " fixed frame " << endl;
               // frameFixedTransform(miu_state_, Fixed_mark_ID_); //TODO:

                //添加图像节点
                cout << " add one node " << endl;
                RobNode robot_node;
                robot_node.id = index_node;   // 节点  id
                robot_node.robot_pos.x = miu_state_.at<float>(0);// 节点  机器人位置
                robot_node.robot_pos.y = miu_state_.at<float>(1);
                robot_node.robot_pos.z = miu_state_.at<float>(2);
                robot_node.pic_node = pic_node;    // 节点  图像块

                robot_node.robot_pos_odom.x = robot_odom_.x;
                robot_node.robot_pos_odom.y = robot_odom_.y;
                robot_node.robot_pos_odom.z = robot_odom_.theta;
                robot_nodes_.push_back(robot_node);  //添加节点集

                //  set g2o  node and sides
                setG2oInfo(miu_state_, index_node);

                //闭环检测
//                cout << " Check loop closure " << endl;
//                int id_matched = checkLoopClosure(robot_nodes_, pic_node, 20, 3*pic_node.num, 4);
//                if( id_matched != -1  )  // 闭环检测完成 有闭环存在
//                {
//                    cout << " loop found" << endl;
//                    Point3f history_pos = robot_nodes_.at(id_matched).robot_pos;
//                    Point3f transform_loop_closure; // 当前的转移到之前的位置
//                    transform_loop_closure.x = miu_state_.at<float>(0) - history_pos.x;
//                    transform_loop_closure.y = miu_state_.at<float>(1) - history_pos.y;
//                    transform_loop_closure.z = miu_state_.at<float>(2) - history_pos.z;

////                    cout << " loop transform" << endl;
////                    systemTransforeNew(miu_state_, transform_loop_closure);  //闭环调整
//                    cout << " g2o graph optimization" << endl;
//                    graphOptimization();
//                }

                landmark_num_ = curr_landmark_num;
                num_EKFSlam_update_++;

                last_rob_pose_.x = miu_state_.at<float>(0);
                last_rob_pose_.y = miu_state_.at<float>(1);
                last_rob_pose_.z = miu_state_.at<float>(2);

                index_node++;      //节点数量增加
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

        miu_convar_p_ =  Mat::zeros(3,3,CV_32FC1);         //这个可以放到初始化函数中去  ？？初值
        miu_convar_p_.at<float>(0,0) = p_init_x_ ;       // 0.1;//0.1;//1;//100;//0.1;
        miu_convar_p_.at<float>(1,1) = p_init_y_ ;       //0.10;//0.1;//1;//100;//0.1;
        miu_convar_p_.at<float>(2,2) = p_init_theta_ ;    //0.38;//0.1;//0.38;

        last_rob_pose_ = Point3f(rob_odom.x, rob_odom.y, rob_odom.theta);//相对位移大小比较的起点
        IS_finished_EKF_init = true;
        cout<<"miu_state_:   "<<miu_state_<<endl;
    }

    time_bag_old = rob_odom.stamp;
    // storeSystemState(miu_state_);
}

/**
 * @brief QrSlam::motionModelNew
 * 将系统运动模型更新,运动预测时系统状态不变.
 * @param motion               输入线速度与角速度(v,w)
 * @param system_state         系统状态量
 * @param system_state_convar  系统协方差量
 * @param variable_num         系统状态量数量
 * @param delta_time           时间间隔
 */
Point3f CVSlam::motionModelNew(Point2f motion, Mat& system_state, Mat& system_state_convar, int variable_num, float delta_time)
{
    Point2f VEL;
    VEL.x = motion.x;
    VEL.y = motion.y;  //  取y  标准正向

    int state = 0;
    if ( abs(VEL.x) < 1  && abs(VEL.y) > 0.05 ) // 1 度/s  自转
        state = 1;//v
    else if ( abs(VEL.x) > 1  && abs(VEL.y) < 0.05 )  // 直线
        state = 2;//v
    else  if ( abs(VEL.x) > 1  && abs(VEL.y) > 0.05 )  //弧线
        state = 3;//v
    else
        state = 0;   // 静止

    Mat miu_increase = Mat::zeros(3, 1, CV_32FC1); //算法第三行计算xPred_SLAM时的3*1矩阵
    float last_miu_theta = miu_state_.at<float>(2) ;//上一个miu_SLAM的角度
    //angleWrap(last_miu_theta);
    Mat Fx = Mat::zeros(3, 3 + 3*variable_num, CV_32FC1);
    Fx.at<float>(0,0) = 1.0;
    Fx.at<float>(1,1) = 1.0;
    Fx.at<float>(2,2) = 1.0;

    //计算Gt   Jacibi_x(x,y,theita)
    Mat Gt = Mat::zeros(3 + 3*variable_num, 3+3*variable_num, CV_32FC1);
    Mat Gt_increase = Mat::zeros(3, 3, CV_32FC1); //用来计算Gt的3*3矩阵
    Mat I_SLAM = Mat::eye(3 + 3*variable_num, 3+3*variable_num, CV_32FC1); //算法中的单位矩阵
    Mat Vt = Mat::zeros(3,2,CV_32FC1);//计算Vt   Jacibi_u(v,w)
    Mat Mt = Mat::zeros(2,2,CV_32FC1);
    Mt.at<float>(0,0) = a1_*VEL.x*VEL.x + a2_*VEL.y*VEL.y;
    Mt.at<float>(1,1) = a3_*VEL.x*VEL.x + a4_*VEL.y*VEL.y;
    Mat Rt = Mat::zeros(3,3,CV_32FC1); //vv

    switch(state)
    {
    case 1: // 0.1 度/s  自转
    {
        cout<<"system_state1:   "<<system_state<<endl;
        miu_increase.at<float>(2) =   VEL.y * delta_time;
        system_state = system_state + Fx.t()*miu_increase;  // X'= X +Jaci_f(x)*delt(x)   predicted mean
        angleWrap(system_state.at<float>(2));
        cout<<"system_state2:   "<<system_state<<endl;
        Gt_increase.at<float>(0,2) = 0;
        Gt_increase.at<float>(1,2) = 0;
        Gt = I_SLAM + Fx.t() * Gt_increase*Fx ;

        Vt.at<float>(2,1) = delta_time;
        Rt = Vt * Mt * Vt.t();//计算Rt

        system_state_convar = Gt * system_state_convar * Gt.t() + Fx.t() * Rt * Fx; //计算预测方差 Px
        break;
    }
    case 2:// 直线
    {
        cout<<"system_state1:   "<<system_state<<endl;
        miu_increase.at<float>(0) =  VEL.x*delta_time * cos(last_miu_theta);
        miu_increase.at<float>(1) =  VEL.x*delta_time * sin(last_miu_theta);
        system_state = system_state + Fx.t()*miu_increase;  // X'= X +Jaci_f(x)*delt(x)   predicted mean
        cout<<"system_state2:   "<<system_state<<endl;

        angleWrap(system_state.at<float>(2));
        Gt_increase.at<float>(0,2) = -VEL.x * sin(last_miu_theta) ;
        Gt_increase.at<float>(1,2) =  VEL.x * cos(last_miu_theta) ;
        Gt = I_SLAM + Fx.t() * Gt_increase*Fx ;

        Vt.at<float>(0,0) =   delta_time * cos(last_miu_theta);
        Vt.at<float>(1,0) =   delta_time * sin(last_miu_theta);
        Rt = Vt * Mt * Vt.t();//计算Rt
        system_state_convar = Gt * system_state_convar * Gt.t() + Fx.t() * Rt * Fx; //计算预测方差 Px

        break;
    }
    case 3://弧线
    {
        //speed mode motion increase   Wd 不能为 0   //TODO::
        cout<<"system_state1:   "<<system_state<<endl;
        miu_increase.at<float>(0) =  -VEL.x/VEL.y * sin(last_miu_theta) + VEL.x/VEL.y * sin(last_miu_theta + VEL.y * delta_time);
        miu_increase.at<float>(1) =   VEL.x/VEL.y * cos(last_miu_theta) - VEL.x/VEL.y * cos(last_miu_theta + VEL.y * delta_time);
        miu_increase.at<float>(2) =   VEL.y * delta_time;
        cout<<"system_state2:   "<<system_state<<endl;

        system_state = system_state + Fx.t()*miu_increase;  // X'= X +Jaci_f(x)*delt(x)   predicted mean
        angleWrap(system_state.at<float>(2));
        Gt_increase.at<float>(0,2) = -VEL.x/VEL.y * cos(last_miu_theta) + VEL.x/VEL.y * cos(last_miu_theta+VEL.y * delta_time);
        Gt_increase.at<float>(1,2) = -VEL.x/VEL.y * sin(last_miu_theta) + VEL.x/VEL.y * sin(last_miu_theta+VEL.y * delta_time);
        Gt = I_SLAM + Fx.t() * Gt_increase*Fx ;

        Vt.at<float>(0,0) = (-sin(last_miu_theta) + sin(last_miu_theta+VEL.y * delta_time))/VEL.y;
        Vt.at<float>(0,1) = VEL.x*(sin(last_miu_theta)-sin(last_miu_theta+VEL.y * delta_time))/VEL.y/VEL.y + VEL.x * cos(last_miu_theta+VEL.y * delta_time)*delta_time/VEL.y;
        Vt.at<float>(1,0) = (cos(last_miu_theta) - cos(last_miu_theta+VEL.y * delta_time))/VEL.y;
        Vt.at<float>(1,1) = -VEL.x*(cos(last_miu_theta)-cos(last_miu_theta+VEL.y * delta_time))/VEL.y/VEL.y+VEL.x * sin(last_miu_theta+VEL.y * delta_time)*delta_time/VEL.y;
        Vt.at<float>(2,0) = 0;
        Vt.at<float>(2,1) = delta_time;
        Rt = Vt * Mt * Vt.t();//计算Rt

        system_state_convar = Gt * system_state_convar * Gt.t() + Fx.t() * Rt * Fx; //计算预测方差 Px
        break;
    }
    default:  //静止
        cout<<"miu_increase1:   "<<system_state<<endl;
        break;
    }

    Point3f robot_increase;
    robot_increase.x = system_state.at<float>(0) - last_rob_pose_.x ;
    robot_increase.y = system_state.at<float>(1) - last_rob_pose_.y ;
    robot_increase.z = system_state.at<float>(2) - last_rob_pose_.z ;

    return robot_increase;
}

int CVSlam::findLandmarkId(int id)
{
    // Lm_observed_Num  vector 里面元素值为id, 其排列顺序为,系统状态landmark的排列顺序.
    for (int j=0; j< index_landmarks_.size();j++ )
    {
        if(id == index_landmarks_.at(j))
            return j;
    }
    return -1;
}

OdomMessage CVSlam::getOdomDataNew(const nav_msgs::Odometry::ConstPtr& odomMsg)
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
    OdomMessage robot_state_info;
    robot_state_info.stamp = odomMsg->header.stamp;
    robot_state_info.theta = yaw;
    robot_state_info.x = odomMsg->pose.pose.position.x*100;
    robot_state_info.y = odomMsg->pose.pose.position.y*100;
    robot_state_info.v = odomMsg->twist.twist.linear.x*100;
    robot_state_info.w = odomMsg->twist.twist.angular.z;
//    robot_state_info.w = -1.0*odomMsg->twist.twist.angular.z;

    //odom_messages_vector.push_back(robot_state_info);  // 里程计数据向量表
    is_odom_update_ = true;
    return robot_state_info;
}
bool CVSlam::addInitPos(cv::Mat img, const OdomMessage robot_state, const int mark_ID, const int num_FULL)
{
    vector<CPointsFour> marks_pix = imgDetectMarks(img);
    if(marks_pix.empty())
        return false;
    cout<<"==== marks_pix.size() "<< marks_pix.size()<<"     "<<endl;
    saveMarksPerImgPix(marks_pix, fQrPixData);

    vector<CPointsFour> marks_metric = pixToMetric(marks_pix);
    saveMarksPerImgMetric(marks_metric, fQrMeterData);
    //saveMeterDatas(landmark5_meter_vector_side_,fQrMeterDataSide);

    cout<<"  "<<robot_state.x<<"  "<<robot_state.y<<"  "<<endl;
    if ( abs(robot_state.x)<10 && abs(robot_state.y)<10 )  //TODO:  起始 2cm 以内
    {
        cout<<" robot keep static "<<endl;
        for (int i=0; i<marks_metric.size();++i )
        {
            CPointsFour mark_2d = marks_metric.at(i);
            if (mark_2d.id == mark_ID )
            {
                mark5_init_vector_.push_back(mark_2d);
                cout<<"Add one mark for init: "<<"   "<<mark5_init_vector_.size()<<endl;
                break;
            }
        }
    }
    if ( num_FULL == mark5_init_vector_.size() )
      return true;
    return false;
}
cv::Mat CVSlam::getSystemState(void )
{
    return miu_state_;
}
Point3f CVSlam::initFrameDiff(const vector<CPointsFour> init_marks_metric,const CPointsFourWorld origin_world)
{
    CPointsFour mark_2d = pQrDetect_->averageCPointsFour(init_marks_metric);

    Point3f diff_data = diffCoordinate(mark_2d, origin_world);
    return diff_data;
}
vector<CPointsFour>  CVSlam::imgProcessNew( cv::Mat& img, bool& Is_Gobal_Search, Point3f& best_mark_info, SearchInfo& search_info)
{
    vector<CPointsFour> marks_pix;

    struct timeval tim1;
    struct timeval tim2;
    gettimeofday(&tim1, NULL);

    if(Is_Gobal_Search)
    { //TODO global
        search_info.num_global_try++;
        marks_pix = imgDetectMarks(img);
        if(!marks_pix.empty())
        {
            Is_Gobal_Search = evaluateNew(marks_pix, best_mark_info);
            search_info.num_global++;
            gettimeofday(&tim2, NULL);
            search_info.time_global = (tim2.tv_sec - tim1.tv_sec)*1000000.0 + (tim2.tv_usec - tim1.tv_usec);//算法耗时说明 微秒
        }
        else
            search_info.time_global = -1.0;
    }
    else
    {  //TODO local
        search_info.num_local_try++;
        cout<<" local search "<<endl;
        cv::Mat system_state = getSystemState();//miu_state
        cv::Mat local_img = inverseObservationModel(img, best_mark_info, system_state) ; //system_state  系统状态量
        cut_img_->convertOnce( local_img); //发布局部图片;
        cout<<" local show "<<endl;
        cout<<" id local_img "<<best_mark_info.z<<"  "<<local_img.cols<<"  "<<local_img.rows<<"   "<<endl;
        cout<<" best_local_mark_center_ "<<best_mark_info.x<<"  "<<best_mark_info.y<<"   "<<endl;

        marks_pix = imgDetectMarks(local_img, best_mark_info);

        if(!marks_pix.empty())
        {
            Is_Gobal_Search = evaluateNew(marks_pix, best_mark_info);
            search_info.num_local++;
            gettimeofday(&tim2, NULL);
            search_info.time_local = (tim2.tv_sec - tim1.tv_sec)*1000000.0 + (tim2.tv_usec - tim1.tv_usec);//算法耗时说明 微秒
        }
        else
            search_info.time_local = -1.0;
    }
    return marks_pix;
}
vector<Point4fffi> CVSlam::getObservationsNew( vector<CPointsFour> marks_metric )
{
    // QrLandMark landmarktemp;
    Point4fffi   mark_one;
    CPointsFour code_one;
    vector<Point4fffi> marks_observed;
    for (vector<CPointsFour>::size_type i=0; i < marks_metric.size(); i++)
    {
        code_one = marks_metric.at(i);  //观测实际值
        mark_one = addObservationPosNew(code_one, code_one.id) ;
        marks_observed.push_back(mark_one); //landmark的index存储在Point3f的第三维中
    }
    return marks_observed;
}

Point4fffi CVSlam::addObservationPosNew(CPointsFour code_one ,int id )
{
    Point2f delta;
    float dst;   //特征距离
    float theta; //特征角w.r.t.robot frame
    Point4fffi mark_one;
    delta = Point2f(code_one.center.X,code_one.center.Y); //-Point2f(RobotPos.X,RobotPos.Y); //已经是delta值了
    //    dst = norm(delta);
    dst = sqrt( delta.x * delta.x + delta.y * delta.y) ;
    theta = atan2(delta.y, delta.x); //- robot_info_.Theta ;//    ??? the true path of vehicle
    angleWrap(theta);
    mark_one.x = dst;
    mark_one.y = theta;
    mark_one.id = id; //temp value
    mark_one.theta = 1.0*(  atan2(  (code_one.corn2.Y - code_one.corn1.Y ),(code_one.corn2.X - code_one.corn1.X ))
                             +atan2( (code_one.corn3.Y - code_one.corn0.Y ),(code_one.corn3.X - code_one.corn0.X ))
                             )/2;
    angleWrap(theta);
    return mark_one;
}
int  CVSlam::getLandmarkNumNew(const vector<Point4fffi> curr_marks_observed, vector<int>& landmark_ids)
{
    for(int i=0; i<curr_marks_observed.size(); i++)
    {
        int Qid = curr_marks_observed.at(i).id;
        bool Is_found = false;
        for(int c=0; c<landmark_ids.size(); c++)
        {
            if(Qid == landmark_ids.at(c))  //出现过
            {
                Is_found = true;
                break;
            }
        }
        if(!Is_found)
            landmark_ids.push_back(Qid);  //
    }
    return landmark_ids.size();
}

void CVSlam::updateSLAM(Mat& sys_state, Mat& sys_convar, int landmarks_num, vector<Point4fffi>& curr_marks, vector<int>& index_landmarks)
{
    Mat Q_convar_ = Mat::zeros(3, 3, CV_32FC1);
    Q_convar_.at<float>(0,0) = convar_measure3_;
    Q_convar_.at<float>(1,1) = convar_measure4_;
    Q_convar_.at<float>(2,2) = 0.05;

    Mat Identity_system = Mat::eye(3+3*landmarks_num, 3+3*landmarks_num, CV_32FC1); //算法中的单位矩阵

    for (int i=0; i<curr_marks.size(); i++)
    {
        int   select_mark_id = curr_marks.at(i).id;  //mark的ID
        Point3f curr_observation = Point3f(curr_marks.at(i).x, curr_marks.at(i).y, curr_marks.at(i).theta); //观测值  是极坐标
        bool Is_landmark = false;
        int curr_landmark_index = -1;
        for (int index=0; index<index_landmarks.size(); ++index)
        {
            if (select_mark_id == index_landmarks.at(index))
            {
                Is_landmark = true;
                curr_landmark_index = index;//  选取第curr_landmark_index个landmark导入观察模型并进行状态更新。
                break;
            }
        }
         //Is_landmark = false; //TODO
        if (Is_landmark)   //
        {
            Point2f delta = Point2f(sys_state.at<float>(3*curr_landmark_index+3), sys_state.at<float>(3*curr_landmark_index+4))
                    - Point2f(sys_state.at<float>(0),sys_state.at<float>(1));
            float distance_2 = delta.x * delta.x + delta.y * delta.y ;
            float distance = sqrt(distance_2);

            Point3f predict_observation;
            predict_observation.x = distance;
//            predict_observation.y = atan2(delta.y, delta.x) - sys_state.at<float>(2);      //偏离robot的方向角方向的角度   相对xy坐标系下值
//            predict_observation.z = sys_state.at<float>(3*curr_landmark_index+5) - sys_state.at<float>(2);
            predict_observation.y = atan2(delta.y, delta.x) - sys_state.at<float>(2);      //偏离robot的方向角方向的角度   相对xy坐标系下值
//            predict_observation.y = atan2(delta.y, delta.x) + sys_state.at<float>(2);      //偏离robot的方向角方向的角度   相对xy坐标系下值

            predict_observation.z = sys_state.at<float>(3*curr_landmark_index+5) + sys_state.at<float>(2);
           // predict_observation.z = sys_state.at<float>(3*curr_landmark_index+5) - sys_state.at<float>(2);

            angleWrap(predict_observation.y);
            angleWrap(predict_observation.z);

            //计算Fj
            Mat F_system = Mat::zeros(6, 3+3 * landmarks_num, CV_32FC1);
            F_system.at<float>(0,0) = 1;
            F_system.at<float>(1,1) = 1;
            F_system.at<float>(2,2) = 1;
            F_system.at<float>(3,3 * curr_landmark_index+3) = 1;
            F_system.at<float>(4,3 * curr_landmark_index+4) = 1;
            F_system.at<float>(5,3 * curr_landmark_index+5) = 1;

            //计算Htjaccibi
            Mat H_i = Mat::zeros(3, 6, CV_32FC1); //用来计算Ht的2*5矩阵
            H_i.at<float>(0,0) = -delta.x * distance;
            H_i.at<float>(0,1) = -delta.y * distance;
            H_i.at<float>(0,3) =  delta.x * distance;
            H_i.at<float>(0,4) =  delta.y * distance;

            H_i.at<float>(1,0) =  delta.y;
            H_i.at<float>(1,1) = -delta.x;
//            H_index.at<float>(1,2) =  distance_2;
            H_i.at<float>(1,2) = -distance_2;
            H_i.at<float>(1,3) = -delta.y;
            H_i.at<float>(1,4) =  delta.x;

//            H_index.at<float>(2,3) = -distance_2;
            H_i.at<float>(2,3) =  distance_2;
            H_i.at<float>(2,5) =  distance_2;

            H_i = (1/distance_2) * H_i;

            Mat H_system = Mat::zeros(3, 3 + 3*landmarks_num, CV_32FC1);
            H_system = H_i * F_system ;

            Mat S_update = Mat::zeros(3, 3, CV_32FC1); // 新息阵
            S_update = H_system * sys_convar * H_system.t() + Q_convar_;

            Mat Kt = Mat::zeros(3 + 3*landmarks_num, 3, CV_32FC1); // klaman增益
            Kt = sys_convar * H_system.t() * S_update.inv();

            Point3f diff_observatin = curr_observation-predict_observation;       //  更新的处理
            angleWrap(diff_observatin.y);
            angleWrap(diff_observatin.z);

            Mat delta_z= Mat::zeros(3,1,CV_32FC1);
            delta_z.at<float>(0) = diff_observatin.x;
            delta_z.at<float>(1) = diff_observatin.y;
            delta_z.at<float>(2) = diff_observatin.z;

            sys_state  =  sys_state + Kt * delta_z;  // xPred_SLAM 关于landmark为极坐标值
            angleWrap(sys_state.at<float>(2));
            angleWrap(sys_state.at<float>(3 * curr_landmark_index+5));

            Point3f robot_point;
            robot_point.x = sys_state.at<float>(0);
            robot_point.y = sys_state.at<float>(1);
            robot_point.z = sys_state.at<float>(2);

            Point3f map_point;
            map_point.x = sys_state.at<float>(3*curr_landmark_index+3);
            map_point.y = sys_state.at<float>(3*curr_landmark_index+4);
            map_point.z = sys_state.at<float>(3*curr_landmark_index+5);

            sys_convar =  (Identity_system - Kt * H_system) * sys_convar;
        }
        else
        {
            index_landmarks.push_back(select_mark_id) ;
            curr_landmark_index = index_landmarks.size()-1 ;
//         if(select_mark_id == 0)
//             curr_landmark_index = 1;
//         if(select_mark_id == 5)
//             curr_landmark_index = 0;



            //                if( fixed_mark_id == Fixed_mark_ID_ )
            //                    Fixed_landmark_index_ = curr_landmark_index;
            // 在观测函数 x y 极坐标系下的值。
            Point3f robot_point;
            robot_point.x = sys_state.at<float>(0);
            robot_point.y = sys_state.at<float>(1);
            robot_point.z = sys_state.at<float>(2);

//            sys_state.at<float>(3*curr_landmark_index+3) = sys_state.at<float>(0) + curr_observation.x * cos( curr_observation.y + sys_state.at<float>(2) );  //第j个landmark的y坐标
//            sys_state.at<float>(3*curr_landmark_index+4) = sys_state.at<float>(1) + curr_observation.x * sin( curr_observation.y + sys_state.at<float>(2) );  //第j个landmark的x坐标
//            sys_state.at<float>(3*curr_landmark_index+5) = sys_state.at<float>(2) + curr_observation.z;  //第j个landmark的x坐标

//            Point2f increase(curr_observation.x * cos(curr_observation.y), curr_observation.y * sin(curr_observation.y) );
//            Point2f increase_xy;
//            //   cos(-z)  sin(-z)   ===>  cos( z)  -sin(z)
//            //   -sin(-z)  cos(-z)  ===>  sin(z)   cos(z)
//            increase_xy.x =  increase.x * cos(sys_state.at<float>(2)) - increase.y * sin(sys_state.at<float>(2)) ;
//            increase_xy.y =  increase.x * sin(sys_state.at<float>(2)) - increase.y * cos(sys_state.at<float>(2)) ;

//            sys_state.at<float>(3*curr_landmark_index+3) = sys_state.at<float>(0) + increase_xy.x;  //第j个landmark的x坐标
//            sys_state.at<float>(3*curr_landmark_index+4) = sys_state.at<float>(1) + increase_xy.y;  //第j个landmark的y坐标

            sys_state.at<float>(3*curr_landmark_index+3) = sys_state.at<float>(0) + curr_observation.x * cos( sys_state.at<float>(2) + curr_observation.y );  //第j个landmark的y坐标
            sys_state.at<float>(3*curr_landmark_index+4) = sys_state.at<float>(1) + curr_observation.x * sin( sys_state.at<float>(2) + curr_observation.y );  //第j个landmark的x坐标

            sys_state.at<float>(3*curr_landmark_index+5) = sys_state.at<float>(2) + curr_observation.z;  //第j个landmark的x坐标


            Point3f map_point;
            map_point.x = sys_state.at<float>(3*curr_landmark_index+3);
            map_point.y = sys_state.at<float>(3*curr_landmark_index+4);
            map_point.z = sys_state.at<float>(3*curr_landmark_index+5);
            map_point.z = sys_state.at<float>(3*curr_landmark_index+5);
        }
    }//end for  curr_marks
    sys_convar =  0.5*(sys_convar + sys_convar.t());
}

vector<int>  CVSlam::getLandmarksIndex()
{
    return   index_landmarks_;
}

int  CVSlam::getFixedLandmarkIndex(const int fixed_mark_id)
{
    int fixed_landmark_index = -1;
    vector<int>  landmarks_index = getLandmarksIndex();
    for(int i=0; i<landmarks_index.size(); i++)
    {
        if( fixed_mark_id == landmarks_index.at(i)  )
        {
            fixed_landmark_index = i;
            break;
        }
    }
    return fixed_landmark_index;
}

int CVSlam::getLandmarksNum()
{
    return index_landmarks_.size();
}

void CVSlam::getOdomDataByAcc(const nav_msgs::Odometry::ConstPtr& odomMsg)
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

    odom_messages_vector.push_back(robot_odom_);  // 里程计数据向量表
    is_odom_update_ = true;
    odom_stamp_old_ = odom_stamp_;

}
Point3f CVSlam::odomModel(Point2f motion, Point3f robotPose,float delta_time)
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
    angleWrap(miu_increase.z);

    return miu_increase;
}
cv::Mat CVSlam::getCovMatrix(const cv::Mat state_matrix, int i, int j)
{
    cv::Mat covMatrix = cv::Mat::zeros(2, 2, CV_32FC1);
    covMatrix.at<float>(0,0) = state_matrix.at<float>(i,i) ;
    covMatrix.at<float>(0,1) = state_matrix.at<float>(i,j) ;
    covMatrix.at<float>(1,0) = state_matrix.at<float>(j,i) ;
    covMatrix.at<float>(1,1) = state_matrix.at<float>(j,j);
    return covMatrix;
}
/**
 * @brief QrSlam::genGaussianValue
 * 高斯扰动量生成
 * @param Sigma2
 * @return
 */
float CVSlam::genGaussianValue(float Sigma2)
{///Generate a Gaussian value with variance sigma
    return rng.gaussian(sqrt(Sigma2));
}

//===================  闭环检测 与 g2o优化 ===========================
PicNode CVSlam::getPicInfo(const vector<CPointsFour>  marks_pix)
{
    PicNode pic_node;
    for(int i=0; i<marks_pix.size(); i++)
    {
        PicInf pic_inf;
        pic_inf.u = marks_pix.at(i).center.X;
        pic_inf.v = marks_pix.at(i).center.Y;
        pic_inf.id = marks_pix.at(i).id;
        pic_node.infs.push_back(pic_inf);
    }
    pic_node.num = pic_node.infs.size();
    return pic_node;
}
int CVSlam::checkLoopClosure(const vector<RobNode> history_nodes,const PicNode current_pic_node, int num, int min_score_pic, int min_value_mark)
{//TODO
    for(int i=0; i<num; i++)  //随机次数
    {
        //int k = rand()/(RAND_MAX + 1.0)*(history_nodes.size()-1);
        int k = rand() % history_nodes.size();
        if( k == 0 ) return -1;
        RobNode history_node = history_nodes.at(k);
        int score = getMatchScore(history_node.pic_node, current_pic_node, min_value_mark ); //获取两张图片的匹配分数
        if( (score <= min_score_pic) && (score != -1) )  //匹配成功
        {
            return k;    //  与第K关键帧重合.
        }
    }
    return -1;
}

void CVSlam::setG2oInfo(const Mat sys_state, const int index_g2o_node)
{
    static Point3f  last_node_pos(0.0, 0.0, 0.0);
    //static int index_g2o_node = 0;
    static bool Is_first_node = true;
    if(Is_first_node)
    {
        g2o::VertexSE2* robot =  new g2o::VertexSE2();
        robot->setId(index_g2o_node);
        robot->setEstimate( Eigen::Isometry2d::Identity() );
        robot->setFixed(true);
        optimizer_.addVertex(robot);

        Is_first_node = false;
    }
    else
    {
        VertexSE2* robot =  new VertexSE2;

        robot->setId(index_g2o_node);
        robot->setEstimate( Eigen::Isometry2d::Identity() );
        optimizer_.addVertex(robot);
        // 里程边
        g2o::EdgeSE2* odom_edge = new g2o::EdgeSE2;
        odom_edge->vertices() [0] = optimizer_.vertex( index_g2o_node-1 );
        odom_edge->vertices() [1] = optimizer_.vertex( index_g2o_node );

        Point3f increase;
        increase.x =  sys_state.at<float>(0) - last_node_pos.x; //deta_x
        increase.y =  sys_state.at<float>(1) - last_node_pos.y; //deta_y
        increase.z =  sys_state.at<float>(2) - last_node_pos.z; //deta_theta

        //  测量矩阵
        SE2 measurement = SE2(increase.x, increase.y, increase.z);
        odom_edge->setMeasurement(measurement);

        //  信息矩阵
        Eigen::Matrix3d information = getRobotInformation();
        odom_edge->setInformation( information );

        optimizer_.addEdge(odom_edge);
    }
    //index_g2o_node += 1;
    last_node_pos.x = sys_state.at<float>(0);
    last_node_pos.y = sys_state.at<float>(1);
    last_node_pos.z = sys_state.at<float>(2);
}
void CVSlam::graphOptimization()
{
    // 优化所有边
    cout<<" optimizing pose graph, vertices: "<< optimizer_.vertices().size() <<endl;
    optimizer_.save("./g2o_output/before.g2o");
    optimizer_.initializeOptimization();
    optimizer_.optimize( 100 ); //可以指定优化步数
    optimizer_.save( "./g2o_output/after.g2o" );
    cout<<" Optimization done."<<endl;
    //optimizer_.clear();
}

Eigen::Matrix3d CVSlam::getRobotInformation()
{
    Eigen::Matrix3d covariance;
    covariance.fill(0.);
    covariance(0, 0) = (double)miu_convar_p_.at<float>(0,0);
    covariance(1, 1) = (double)miu_convar_p_.at<float>(1,1);
    covariance(2, 2) = (double)miu_convar_p_.at<float>(2,2);
    return covariance.inverse();
}

//====================图像处理==================
vector<CPointsFour> CVSlam::imgDetectMarks(cv::Mat image )
{
    vector<CPointsFour> Marks_pix = pQrDetect_->detectMarks(image, visual_mark_num_, true);
    return Marks_pix;
}
vector<CPointsFour> CVSlam::imgDetectMarks(cv::Mat image, Point3f mark_center)
{
    vector<CPointsFour> Marks_pix = pQrDetect_->detectMarks(image, visual_mark_num_, false);
    for(int i=0; i<Marks_pix.size(); i++)
    {
        Marks_pix.at(i).addOffset(mark_center.x, mark_center.y);
    }
    return Marks_pix;
}
vector<CPointsFour> CVSlam::pixToMetric(vector<CPointsFour> pix_vector)
{
    vector<CPointsFour> metric_vector;
    for(vector<CPointsFour>::size_type i=0; i<pix_vector.size(); i++ )
    {
        CPointsFour  dst;
        //CPointsFour  dst_side;
        CPointsFour  src = pix_vector.at(i);
        pQrDetect_->imgCornerToWorld( dst, src);         // 利用已知高度信息   //TODO
        metric_vector.push_back(dst);
        // pQrDetect_->pixToMeter( dst_side, src);         //  利用已知边长信息
        //landmark5_meter_vector_side_.push_back(dst_side);
    }
    return metric_vector;
}
cv::Mat CVSlam::inverseObservationModel(cv::Mat& full_img, Point3f& mark_center, Mat system_state)
{
    Point3f pos_world ;
    Point2f landmark_world;
    Point3f observe_increase_world;   //(dx dy seita)
    Point2f observe_increase_camera,observe_increase_pix;

    pos_world.x = system_state.at<float>(0);
    pos_world.y = system_state.at<float>(1);
    pos_world.z = system_state.at<float>(2);
    cout<<" pos_world "<<pos_world.x<<" "<<pos_world.y<<"  "<<pos_world.z<<endl;

    int index_landmark = findLandmarkId(mark_center.z);             // 系统状态量序号
    cout<<" "<<index_landmark<<endl;
    landmark_world.x = system_state.at<float>(3+3*index_landmark);
    landmark_world.y = system_state.at<float>(4+3*index_landmark);
    cout<<" landmark_world "<<landmark_world.x<<" "<<landmark_world.y<<endl;

    observe_increase_world.x = landmark_world.x - pos_world.x;
    observe_increase_world.y = landmark_world.y - pos_world.y;
    observe_increase_world.z = pos_world.z;
    cout<<" observe_increase_world "<<observe_increase_world.x<<" "<<observe_increase_world.y<<"  "<<observe_increase_world.z<<endl;

    observe_increase_camera = worldToCamera(observe_increase_world);
    cout<<" observe_increase_camera "<<observe_increase_camera.x<<" "<<observe_increase_camera.y<<endl;

    observe_increase_pix = cameraToImage(observe_increase_camera, mark_center.z);  //id
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

    mark_center.x = cutRectX;
    mark_center.y = cutRectY;

    cv::Mat cut_img;
    full_img(rect).copyTo(cut_img);

    cv::rectangle(full_img,rect,CV_RGB(0,255,0),1,8);
    cv::circle(full_img,cvPoint( mark_center.x,mark_center.y),
               10,CV_RGB(0,255,0),-1); //绘制mark位置

    cv::circle(full_img,cvPoint( mark_center.x+2*best_mark_center_side_,mark_center.y+2*best_mark_center_side_),
               10,CV_RGB(0,0,255),-1); //绘制mark位置

    return cut_img;
}
int CVSlam::getMatchScore(PicNode history_pic_node, PicNode current_pic_node, int min_distance)
{
    int score = 0;
    if(current_pic_node.num == history_pic_node.num) //数量一致
    {
        for(int i=0; i<current_pic_node.num; i++)
        {
            int value = 0;
            PicInf current_mark_one = current_pic_node.infs.at(i);
            for(int j=0; j<history_pic_node.num; j++ )
            {
                PicInf history_mark_one = history_pic_node.infs.at(j);
                if(current_mark_one.id == history_mark_one.id )
                {
                    Point2i error;
                    error.x = current_mark_one.u - history_mark_one.u;
                    error.y = current_mark_one.v - history_mark_one.v;
                    value = sqrt(error.x * error.x + error.y*error.y) ;

                    if(value <= min_distance)  // 每两个二维码间的距离
                        score += value;
                    else
                        return -1;

                    break;
                }
            }
        }
        return score;
    }
    else
        return -1;
}

bool CVSlam::evaluateNew(vector<CPointsFour> marks_pix, Point3f& best_mark_center)
{
    float min = 800.0;
    float Radius ;
    for(int i=0; i<marks_pix.size();i++)
    {
        float distance = sqrt( (marks_pix.at(i).center.X - 320) * (marks_pix.at(i).center.X - 320)
                               + (marks_pix.at(i).center.Y -240 ) *(marks_pix.at(i).center.Y -240 ));
        if( distance < min )
        {
            Radius = sqrt( (marks_pix.at(i).corn0.X - marks_pix.at(i).corn3.X) * (marks_pix.at(i).corn0.X - marks_pix.at(i).corn3.X)
                           + (marks_pix.at(i).corn0.Y - marks_pix.at(i).corn3.Y) * (marks_pix.at(i).corn0.Y - marks_pix.at(i).corn3.Y));
            min = distance;
            best_mark_center.x = marks_pix.at(i).center.X ;
            best_mark_center.y = marks_pix.at(i).center.Y ;
            best_mark_center.z = marks_pix.at(i).id; //注意这选择的是五点法
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
 * @brief QrSlam::getImgData
 *  获取图像信息,当获取的图像有定义编码is_img_update_返回true,否则返回假.
 *  观测到的信息都存储在 landmark5_vector_  ;  信息为沿机器人方向相对中心的(x,y)像素值
 * @param imgMsg
 */
cv::Mat CVSlam::getImgDataNew(const sensor_msgs::ImageConstPtr& imgMsg)
{
    image_stamp_ = imgMsg->header.stamp;
    cv_bridge::CvImagePtr  cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(imgMsg,sensor_msgs::image_encodings::BGR8);
    //cv_ptr->image.copyTo(cv_camera_);
    cv::Mat img = cv_ptr->image ;
    if ( img.empty() )
        is_img_update_ = false ;
    else
        is_img_update_ = true;

    return img;
}

//=====================绘图相关==================
void CVSlam::pubSystemStates()
{
    cv_slam::systemState sys_state;
    cv_slam::odom odom;
    odom.x = robot_odom_.x;
    odom.y = robot_odom_.y;
    odom.theta = robot_odom_.theta;

    cv_slam::robot robot;
    robot.x = miu_state_.at<float>(0);
    robot.y = miu_state_.at<float>(1);
    robot.theta = miu_state_.at<float>(2);

    vector<int> landmarkIDs = getLandmarksIndex();
    for(int i=0; i<landmarkIDs.size(); i++)
    {
        cv_slam::landmark landmark;
        landmark.x = miu_state_.at<float>(3*i +3);
        landmark.y = miu_state_.at<float>(3*i +4);
        landmark.theta = miu_state_.at<float>(3*i +5);
        landmark.id = landmarkIDs.at(i);
        sys_state.landmark.push_back(landmark);
    }

    sys_state.odom = odom;
    sys_state.robot_state = robot;

    systemStates_.publish(sys_state);
}


void CVSlam::showImageNew()
{
    showRobot(raw_global_map_, robot_odom_, CV_RGB(0,0,0)) ;
    showSystemStateRobotNew(raw_global_map_, miu_state_, CV_RGB(0,0,255));
    drawCoordinate(raw_global_map_);
    fDebug<<robot_odom_.x<<"  "<<robot_odom_.y<<"  "<<robot_odom_.theta<<"  "
          <<miu_state_.at<float>(0) <<"  "<<miu_state_.at<float>(1)<<"  "<<miu_state_.at<float>(2)<<"  "<<endl;

    raw_global_map_.copyTo( global_map_display_ );    //避免历史重影 landmark
    showSystemStateLandmarkNew( global_map_display_, miu_state_, CV_RGB(255,0,0));

    resize(global_map_display_, map_, map_size_);
    slam_img_cvt_->convertOnce( map_);

    //对比显示 robot
    showRobotOrientationNew(cv_camera_, robot_odom_, CV_RGB(0,0,0),50,50);
    showRobotOrientationNew(cv_camera_, miu_state_, CV_RGB(0,0,0),300,50);
    raw_img_cvt_->convertOnce(cv_camera_);
}

void CVSlam::showSystemStateRobotNew(cv::Mat& map, const Mat miu_state, const Scalar rgb)
{
    int temp_X = miu_state.at<float>(0) + map_base_x_;
    int temp_Y = miu_state.at<float>(1) + map_base_y_;
    temp_Y = map.rows - temp_Y ;
    cv::circle(map,Point( temp_X,temp_Y),2,rgb,1); //绘制 robot

    cv::Mat rob_motion_cov = getCovMatrix(miu_state, 0, 1);
    cv::Point2f center(temp_X, temp_Y);
    //drawEllipse(map, center, rob_motion_cov, CV_RGB(0, 150,0) );
}

void CVSlam::showSystemStateLandmarkNew(cv::Mat& map, const Mat miu_state, const Scalar rgb)
{
    // cv::Mat map_copy ;
    for (int t = 0; t < id_all_marks_.size(); t++)
    {
        float X= miu_state.at<float>(3+t*3)+ 1000;
        float Y= miu_state.at<float>(4+t*3)+ 1000;

        Y = map.rows - Y ;
        cv::circle(map,Point( X,Y),2,rgb,2); //绘制mark位置

        cv::Mat landmark_cov = getCovMatrix(miu_state, 3+t*3, 4+t*3);
        cv::Point2f center(X,Y);
        // drawEllipse(map, center, landmark_cov, CV_RGB(0, 150,0) );

        std::string text = int2str(id_all_marks_.at(t));
        cv::putText(map,text,Point(X,Y+20),CV_FONT_HERSHEY_COMPLEX, 2, CV_RGB(255, 0,0) );
    }
}

void CVSlam::showRobotOrientationNew(Mat image, OdomMessage rob_odom,Scalar rgb,int x_coordinate,int y_coordinate)
{
    const int ROBOT_DEFAULT_RADIUS = 2;
    const int ROBOT_DEFAULT_ARROW_LEN = 30;

    Point3d robot_pose;
    robot_pose.x = rob_odom.x;
    robot_pose.y = rob_odom.y;
    robot_pose.z = rob_odom.theta;
    //采用地图固定下  初始坐标系调整就可以
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

void CVSlam::showRobotOrientationNew(Mat image, Mat rob_update_mat,Scalar rgb,int x_coordinate,int y_coordinate)
{
    const int ROBOT_DEFAULT_RADIUS = 2;
    const int ROBOT_DEFAULT_ARROW_LEN = 30;

    Point3d robot_pose;
    robot_pose.x = rob_update_mat.at<float>(0);
    robot_pose.y = rob_update_mat.at<float>(1);
    robot_pose.z = rob_update_mat.at<float>(2);

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

    //landmark 数量
    std::string  num_text = int2str(id_all_marks_.size());
    cv::putText(cv_camera_,num_text,Point( 40,40),CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(0, 0,255) );
}

/**
 * @brief QrSlam::displayMatrix
 * Mat 矩阵打印输出
 * @param matrix
 */
void CVSlam::displayMatrix(Mat matrix)
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

void CVSlam::writeMatrix(Mat matrix ,ofstream& f_test)
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
 * @brief QrSlam::showRobot
 * 在mat 图片上绘制机器人位置
 * @param map          图片
 * @param robot_info   机器人信息（x,y,theta）
 * @param rgb          绘制时颜色选择
 */
void CVSlam::showRobot(cv::Mat& map, const OdomMessage rob_odom, const Scalar rgb)
{
    Point3d robot_pose;
    robot_pose.x = rob_odom.x;
    robot_pose.y = rob_odom.y;
    robot_pose.z = rob_odom.theta;

    const int ROBOT_DEFAULT_RADIUS = 2;
    const int ROBOT_DEFAULT_ARROW_LEN = 10;

    Point start, end;
    start.x = robot_pose.x + map_base_x_;
    start.y = robot_pose.y + map_base_y_;
    int thickness = 2;
    int lineType = 8;
    end.x = start.x + ROBOT_DEFAULT_ARROW_LEN * cos(robot_pose.z);
    end.y = start.y + ROBOT_DEFAULT_ARROW_LEN * sin(robot_pose.z);  //display  y  convert ..
    start.y = map.rows - start.y;
    end.y   = map.rows - start.y;

    circle(map,start,ROBOT_DEFAULT_RADIUS,rgb,0,lineType );
    //line( map,start,end,rgb,thickness,lineType );
}
/**
 * @brief QrSlam::showRobotTriangle
 * 在mat 图片上绘制机器人位置，以三角形式表示        -->单独做到一个截框界面显示 每步机器人信息（x,y,theta）
 * @param map          图片
 * @param robot_info   机器人信息（x,y,theta）
 * @param rgb          绘制时颜色选择
 */
void CVSlam::showRobotTriangle(cv::Mat& map, OdomMessage rob_odom, Scalar rgb)
{
    float width = 3;
    Point robot_pos;
    robot_pos.x =  rob_odom.x + map_base_y_ ;
    robot_pos.y =  rob_odom.y + map_base_x_ ;

    float width_cos = width*cos(rob_odom.theta);
    float width_sin = width*sin(rob_odom.theta);

    Point2i end;
    end.x = robot_pos.x + width_cos;
    end.y = robot_pos.x + width_sin;

    int thickness = 1;
    int lineType = 8;

    robot_pos.y = map.rows - robot_pos.y;
    end.y = map.rows - end.y;
    cv::circle(map,robot_pos,3,rgb,1); //绘制mark位置
    line( map,robot_pos,end,rgb,thickness,lineType );
}


void CVSlam::drawCoordinate(cv::Mat& mat)
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

void CVSlam::posesPublishNew()
{
    // 建立tf树转换  odom_to_img_ ， map 到 odom
    // my_world(fixed) --> my_odom
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    odom_to_img_.sendTransform ( tf::StampedTransform( tf::Transform(q, tf::Vector3(0.0, 0.0, 0.0)),
                                                       ros::Time::now(),"my_world", "my_odom")  );
    cout<<"finish tf of odom and img "<<endl;

    visualization_msgs::Marker points;
    points.header.frame_id = "my_world";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
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
    for (int i=0;i<(miu_state_.rows-3)/3;++i)
    {
        geometry_msgs::Point pose_miu_map;
        pose_miu_map.x =  miu_state_.at<float>(3+3*i)/100.0;
        pose_miu_map.y =  miu_state_.at<float>(4+3*i)/100.0;
        pose_miu_map.z =  0.4;
        points.points.push_back(pose_miu_map);
    }
    marker_pub_.publish(points);

    //miu_robot
    nav_msgs::Odometry miu_odom;
    miu_odom.header.stamp = ros::Time::now();
    miu_odom.header.frame_id = "my_world";
    //miu_odom.child_frame_id = "my_odom";
    miu_odom.pose.pose.position.x = miu_state_.at<float>(0)/100.0;
    miu_odom.pose.pose.position.y = miu_state_.at<float>(1)/100.0;
    miu_odom.pose.pose.position.z = 0;

    tf::Quaternion q_miu;
    q_miu.setRPY(0, 0,miu_state_.at<float>(2));
    miu_odom.pose.pose.orientation.x = q_miu.getX();
    miu_odom.pose.pose.orientation.y = q_miu.getY();
    miu_odom.pose.pose.orientation.z = q_miu.getZ();
    miu_odom.pose.pose.orientation.w = q_miu.getW();
    robot_pose_publisher_.publish(miu_odom);

    //odom_robot
    nav_msgs::Odometry robot_odom;
    robot_odom.header.stamp = ros::Time::now();
    robot_odom.header.frame_id = "my_odom";
    //robot_odom.child_frame_id = "my_odom";
    robot_odom.pose.pose.position.x = robot_odom_.x/100.0;
    robot_odom.pose.pose.position.y = robot_odom_.y/100.0;
    robot_odom.pose.pose.position.z = 0;

    tf::Quaternion q_robot;
    q_robot.setRPY(0, 0,robot_odom_.theta);
    robot_odom.pose.pose.orientation.x = q_robot.getX();
    robot_odom.pose.pose.orientation.y = q_robot.getY();
    robot_odom.pose.pose.orientation.z = q_robot.getZ();
    robot_odom.pose.pose.orientation.w = q_robot.getW();
    robot_pose_publisher_.publish(robot_odom);

}

void CVSlam::drawEllipse(cv::Mat img_test, cv::Point2f center, cv::Mat covMartix, const Scalar& color)
{
    Mat eigen_values;
    Mat eigen_vector;
    eigen(covMartix,eigen_values,eigen_vector);
    //Point2f center(50.0,50.0);
    float angle = atan2(eigen_values.at<float>(0,1),eigen_values.at<float>(0,0))*180.0/3.1415;

    float length = max(center.x, center.y);
    Size  size;
    size.width =  int(sqrt(eigen_values.at<float>(0))/1000*img_test.cols);   //如果covMatrix为CV_64FC1型，那么应把float改成double，如下相同
    size.height = int(sqrt(eigen_values.at<float>(1))/300*img_test.rows);
    size.width = max(1, size.width);//*10;
    size.height = max(1, size.height);//*10;

    //    size.width =  length + center.x > img_test.cols ? img_test.cols - center.x-1:size.width;
    //    size.height = length + center.y > img_test.rows ? img_test.rows - center.x-1:size.height;

    //    size.width =  -length + center.x < 0 ? center.x-1:size.width;
    //    size.height = -length + center.y < 0 ? center.y-1:size.height;

    // Size size(eigen_values.at<float>(0)*5,eigen_values.at<float>(1)*5);

    ellipse(img_test, center, size ,angle, 0, 360, color);
}

//=====================数据保存==================
void CVSlam::saveMarksPerImgPix( vector<CPointsFour> vector_data, ofstream&  file )
{
    file  <<  vector_data.size();
    for (vector<CPointsFour>::size_type i=0; i< vector_data.size();i++)
    {
        file  << " " <<  vector_data[i].id  << " " <<  vector_data[i].corn0.X  << " " << vector_data[i].corn0.Y   << " " <<  vector_data[i].corn1.X << " " << vector_data[i].corn1.Y
              << " " <<  vector_data[i].corn2.X  << " " << vector_data[i].corn2.Y   << " " <<  vector_data[i].corn3.X << " " << vector_data[i].corn3.Y
              << " " <<  vector_data[i].center.X << " " << vector_data[i].center.Y;
    }
    file  << " " <<  endl;
}

void CVSlam::saveMarksPerImgMetric( vector<CPointsFour> vector_data,ofstream&  file )
{
    file  <<  vector_data.size();
    for (vector<CPointsFour>::size_type i=0; i< vector_data.size();i++)
    {
        file  << " " <<  vector_data[i].id  << " " <<  vector_data[i].corn0.X  << " " << vector_data[i].corn0.Y   << " " <<  vector_data[i].corn1.X << " " << vector_data[i].corn1.Y
              << " " <<  vector_data[i].corn2.X  << " " << vector_data[i].corn2.Y   << " " <<  vector_data[i].corn3.X << " " << vector_data[i].corn3.Y
              << " " <<  vector_data[i].center.X << " " << vector_data[i].center.Y;
    }
    file  << " " <<  endl;
}

void CVSlam::savePixDatas( vector<CPointsFour> vector_data )
{
    fQrPixData  <<  vector_data.size();
    for (vector<CPointsFour>::size_type i=0; i< vector_data.size();i++)
    {
        fQrPixData  << " " <<  vector_data[i].id  << " " <<  vector_data[i].corn0.X  << " " << vector_data[i].corn0.Y   << " " <<  vector_data[i].corn1.X << " " << vector_data[i].corn1.Y
                    << " " <<  vector_data[i].corn2.X  << " " << vector_data[i].corn2.Y   << " " <<  vector_data[i].corn3.X << " " << vector_data[i].corn3.Y
                    << " " <<  vector_data[i].center.X << " " << vector_data[i].center.Y;
    }
    fQrPixData  << " " <<  endl;
}
void CVSlam::saveMeterDatas( vector<CPointsFour> vector_data,ofstream&  writeData )
{
    writeData  <<  vector_data.size();
    for (vector<CPointsFour>::size_type i=0; i< vector_data.size();i++)
    {
        writeData  << " " <<  vector_data[i].id  << " " <<  vector_data[i].corn0.X  << " " << vector_data[i].corn0.Y   << " " <<  vector_data[i].corn1.X << " " << vector_data[i].corn1.Y
                   << " " <<  vector_data[i].corn2.X  << " " << vector_data[i].corn2.Y   << " " <<  vector_data[i].corn3.X << " " << vector_data[i].corn3.Y
                   << " " <<  vector_data[i].center.X << " " << vector_data[i].center.Y;
    }
    writeData  << " " <<  endl;
}
void CVSlam::saveSystemState(ofstream&  file )
{
    Mat system_state = getSystemState();
    file << " "<< robot_odom_.x << " "<< robot_odom_.y << " " << robot_odom_.theta << " "  ;
    for (int i=0; i<system_state.rows; ++i)
    {
        file << system_state.at<float>(i) << " " ;
    }
    file << endl;
}
void CVSlam::saveInitFrameDiff(const Point3f frame_diff, ofstream&  file )
{
    file<<"x y theta: "<<frame_diff.x <<"  "<<frame_diff.y <<"  "<<frame_diff.z<<" "<<endl;
}

/**
 * @brief QrSlam::storeSystemState
 * 系统状态量向量表保存
 * @param systemState  系统状态
 */
void CVSlam::storeSystemState( Mat systemState)
{
    Point3f robot_pose;
    robot_pose.x = systemState.at<float>(0);
    robot_pose.y = systemState.at<float>(1);
    robot_pose.z = systemState.at<float>(2);

    est_path_points_.push_back(robot_pose);

    Point3f landmark;
    vector<Point3f> landmarks;
    for (vector<int>::size_type t = 0; t < id_all_marks_.size(); t++)
    {
        landmark.x = miu_state_.at<float>(3+t*2);
        landmark.y = miu_state_.at<float>(4+t*2);
        landmark.z = index_landmarks_[t];

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

//=====================坐标变换==================
void CVSlam::systemTransforeNew(Mat& sys_state, Point3f trans)
{
    Point3d dst;   //robot state         //robot_odom_   是里程计算上传式(固定式旋转与平移)非增量计算式, landmark一次调整大,后面变化小.
    dst.x =  cos(trans.z) * sys_state.at<float>(0) + sin(trans.z) * sys_state.at<float>(1) - trans.x;
    dst.y = -sin(trans.z) * sys_state.at<float>(0) + cos(trans.z) * sys_state.at<float>(1) - trans.y;
    dst.z =  sys_state.at<float>(2) - trans.z;

    sys_state.at<float>(0) = dst.x;
    sys_state.at<float>(1) = dst.y;
    sys_state.at<float>(2) = dst.z;

    int total_landmarks = getLandmarksNum();
    for(int i=0; i<total_landmarks; i++)
    {
        dst.x =  cos(trans.z) * sys_state.at<float>(3+3*i) + sin(trans.z) * sys_state.at<float>(4+3*i) - trans.x;
        dst.y = -sin(trans.z) * sys_state.at<float>(3+3*i) + cos(trans.z) * sys_state.at<float>(4+3*i) - trans.y;
        dst.z =  sys_state.at<float>(5+3*i) - trans.z;

        sys_state.at<float>(3+3*i) = dst.x;
        sys_state.at<float>(4+3*i) = dst.y;
        sys_state.at<float>(5+3*i) = dst.z;
    }
}

OdomMessage CVSlam::odomToWorld(const OdomMessage src)
{
    OdomMessage dst;
    dst.init(src);
//    dst.x =  cos(coordinate_angle_) * src.x + sin(coordinate_angle_) * src.y - coordinate_x_;
//    dst.y = -sin(coordinate_angle_) * src.x + cos(coordinate_angle_) * src.y - coordinate_y_;

    dst.x =  cos(coordinate_angle_) * src.x - sin(coordinate_angle_) * src.y - coordinate_x_;
    dst.y =  sin(coordinate_angle_) * src.x + cos(coordinate_angle_) * src.y - coordinate_y_;

    dst.theta =  src.theta - coordinate_angle_;
    return dst;
}

OdomMessage CVSlam::odomToWorld(const OdomMessage src, const Point3f frame_diff)
{
    OdomMessage dst;
    dst.init(src);
//    dst.x =  cos(frame_diff.z) * src.x + sin(frame_diff.z) * src.y - frame_diff.x;
//    dst.y = -sin(frame_diff.z) * src.x + cos(frame_diff.z) * src.y - frame_diff.y;

    dst.x =  cos(frame_diff.z) * src.x - sin(frame_diff.z) * src.y - frame_diff.x;
    dst.y =  sin(frame_diff.z) * src.x + cos(frame_diff.z) * src.y - frame_diff.y;

    dst.theta =  src.theta - frame_diff.z;
    return dst;
}

Point2f CVSlam::worldToCamera(Point3f observe_increase_world)
{
    Point2f observe_increase_camera;
    observe_increase_camera.x = observe_increase_world.x*cos(observe_increase_world.z) -
            observe_increase_world.y*sin(observe_increase_world.z) ;
    observe_increase_camera.y = observe_increase_world.x*sin(observe_increase_world.z) +
            observe_increase_world.y*cos(observe_increase_world.z) ;
    return observe_increase_camera;
}

Point2f CVSlam::cameraToImage(Point2f observe_increase_camera, int id)
{
    Point2f observe_increase_image  ;
    observe_increase_image = pQrDetect_->trueToimPos(observe_increase_camera, id);
    return observe_increase_image;

}

bool CVSlam::frameFixedTransform(Mat& miu_state , const int fixed_mark_id)   //当前系统状态 涉及到的地图与初始原点的对比
{
    int fixed_landmark_index = getFixedLandmarkIndex(fixed_mark_id);

    if(fixed_landmark_index == -1)
        return false;

    Point3f frame_transform;
    frame_transform.x = miu_state.at<float>(3+3*fixed_landmark_index) - 0.0;
    frame_transform.y = miu_state.at<float>(4+3*fixed_landmark_index) - 0.0;
    frame_transform.z = miu_state.at<float>(5+3*fixed_landmark_index) - 0.0;

    systemTransforeNew(miu_state,frame_transform);
    return true;
}

/*
 * Point3f QrSlam::diffCoordinate(CPointsFour mark_2d,CPointsFourWorld mark_2d_world)
 *    初始位置运算，
 * 输入： *     为均值化后的mark， 起始物理位置与世界位置
 * 输出： *     相对偏移值
 */
Point3f CVSlam::diffCoordinate(CPointsFour mark_2d,CPointsFourWorld mark_2d_world)
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
    d_theta = 1.0*(  atan2(  (mark_2d.corn2.Y - mark_2d.corn1.Y ),(mark_2d.corn2.X - mark_2d.corn1.X ))
                     +atan2( (mark_2d.corn3.Y - mark_2d.corn0.Y ),(mark_2d.corn3.X - mark_2d.corn0.X ))
                     )/2;
    coordinate_data.x = d_x;
    coordinate_data.y = d_y;
    coordinate_data.z = d_theta;
    return  coordinate_data;
}

//=====================工具==================

/**
 * @brief QrSlam::angleWrap
 * 角度归一化，规范到（-pi,pi）范围内
 * @param angle
 */
float CVSlam::angleWrap(float& angle)
{
    ///这个函数用来实现把角度规划到-pi至pi之间
    if (angle>=CV_PI)
        while (angle >= CV_PI)
        { angle=angle-2*CV_PI;}
    else if (angle<-1.0*CV_PI)
        while (angle < -1.0*CV_PI)
        { angle = angle+2*CV_PI;}
    return angle;
}
/**
 * @brief QrSlam::int2str
 * int 转换成 string
 * @param num
 * @return
 */
std::string  CVSlam::int2str(int num)
{
    std::stringstream ss;
    ss  <<  num;
    std::string text = ss.str();
    return text;
}
/**
 * @brief QrSlam::float2str
 * 数据类型转换 float 转换到 string
 * @param num
 * @return
 */
std::string  CVSlam::float2str(float num)
{
    std::stringstream ss;
    ss << num;
    std::string text = ss.str();
    return text;
}


