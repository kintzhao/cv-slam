/*
    局部处理与全局处理的不同是针对的处理图像大小不一样。
    一个是在截取的小块图像，一个是在全幅图像中处理。。小块是通过找到最优Ｍark，利用已知位置列表估计所有mark与Mark的相对位置
    反映到实际像素值中。。在截取相对位置可能出现在同一幅图像中的mark作为visualbal_mark.分别截取可见的mark再进行trace.
*/

#include "detectqrcode.h"

ofstream fConpensate_img_process("./OutPut/fConpensate_img_process.txt"); //采集mark的解析值
ofstream fThreshold_img_process("./OutPut/fThreshold_img_process.txt"); //采集mark的解析值

DetctQrcode::DetctQrcode(char * mapFile, int id_fixed_mark)
{
    useBCH_ = false;
    if ( mapFile != NULL)
    {
        printf("configuration filename: %s !", mapFile);
    }
    createMap( input_data_, mapFile);

    if (!readConfigureInfo(60))  //迭代操作  初始化vextor[0-599]:  ht orients xoff yoff sideSizes
    {
        cout<<"read configure erro"<<endl;
        waitKey();
    }
    cout<<"Finish reading the configure message ! "<<endl;   //

    if (!readFixedMarkInfo(id_fixed_mark))
    {
        cout<<"Check the configure of fixed mark :"<<id_fixed_mark<<" !"<<endl;
    }
    cout<<"the id of fixed mark is: "<<fixed_mark_world_.ID <<" !"<<endl;

    undistort_ = atoi( input_data_[std::string("isundistort")].c_str() ) ; //是否加入图像去畸变
    calibFile_ = input_data_[std::string("calibFile")].c_str();
    cv::FileStorage fs2(input_data_[std::string("cvCalibFile")].c_str(), cv::FileStorage::READ);

    //获取标定参数
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    distcoeffs_ = cv::Mat::zeros(5, 1, CV_64F);
    fs2["Camera_Matrix"] >> camera_matrix_;               //相机矩阵
    fs2["Distortion_Coefficients"] >> distcoeffs_;        //畸变系数

    //相机焦距相关
    camInnerPara_.fx = camera_matrix_.at<double>(0,0)/100.0;
    camInnerPara_.fy = camera_matrix_.at<double>(1,1)/100.0;

    if (undistort_)          //    #define undistort 1
    {
        camInnerPara_.dx = camera_matrix_.at<double>(0,2);
        camInnerPara_.dy = camera_matrix_.at<double>(1,2);
    }
    else
    {
        camInnerPara_.dx = img_W_/2;
        camInnerPara_.dy = img_H_/2;
    }

    middle_.x = (int)camInnerPara_.dx;
    middle_.y = (int)camInnerPara_.dy;

    cv::initUndistortRectifyMap( camera_matrix_, distcoeffs_, cv::Mat(),
                                 cv::getOptimalNewCameraMatrix(camera_matrix_, distcoeffs_, cv::Size(img_W_,img_H_), 1, cv::Size(img_W_,img_H_), 0),
                                 cv::Size(img_W_,img_H_),CV_16SC2, map1_, map2_);

    marks_convert_pub__ = new ImageConverter("/detect_qr/qr_img");
}

DetctQrcode::~DetctQrcode()
{
    // cv::imwrite("./2d_mark",show_landmark_img_);
    if (!marks_convert_pub__)
    {
        delete marks_convert_pub__;
    }
}

bool DetctQrcode::readConfigureInfo(int max_num)
{
    MapType::iterator it; //输出相机成像尺寸信息
    for (int hti = 0 ; hti < max_num ; hti++)
    {
        //配置参数字符等价
        std::string htstr = arrToStr("ht",hti);
        std::string orientsstr = arrToStr("orients",hti);
        std::string sideSizesstr = arrToStr("sideSizes",hti);

        MapType::iterator it;
        it = input_data_.find(htstr.c_str());
        if ( it != input_data_.end() )
            ht_.push_back( atof( it->second.c_str() ) ); // Add data to the end of the %vector.
        else
            ht_.push_back(0.0);

        it = input_data_.find(orientsstr.c_str());
        if ( it != input_data_.end() )
            orients_.push_back( atoi( it->second.c_str() ) );
        else
            orients_.push_back(0.0);

        it = input_data_.find(sideSizesstr.c_str());
        if ( it != input_data_.end() )
            side_sizes_.push_back( atoi( it->second.c_str() ) );
        else
            side_sizes_.push_back(0.0);
    }
    if (side_sizes_.empty()||ht_.empty()||orients_.empty())
        return false;
    else //configure no empty
    {
        for ( int hti = 0 ; hti < max_num ; hti++)
        {
            if (ht_[hti]!=0)
            {
                cout<<" Mark Id: "<<hti<<"-- ht["<<hti<<"]="<<ht_[hti]<<"  "<<endl;
            }
        }
    }
    return true;
}

bool DetctQrcode::readFixedMarkInfo(int id)
{
    bool flag = true;
    std::string vertiex_lists[15] = {"corn0x","corn0y","corn0z", "corn1x","corn1y","corn1z",
                                     "corn2x","corn2y","corn2z", "corn3x","corn3y","corn3z",
                                     "centerx","centery","centerz"};
    MapType::iterator it;
    float*  ptr = &fixed_mark_world_.corn0.X;
    for(int i=0; i<15; i++)
    {
        std::string corn = arrToStr(vertiex_lists[i].c_str(),id);
        it = input_data_.find(corn.c_str());
        if ( it != input_data_.end() )
        {
            // fixed_mark_wrs_.corn0.X = atof( it->second.c_str() ) ;
            *ptr = atof( it->second.c_str() ) ;
        }
        else
        {
            //fixed_mark_wrs_.corn0.X = 0.0;
            *ptr =   0.0 ;
            flag = false;
            cout<<"check the cnfigure of fixed mark : "<<id<<" !"<<endl;
        }
        ptr++;
    }

    fixed_mark_world_.ID = id;
    return flag;
}

/*
 * CPointsFourWorld DetctQrcode::getInitMarkMessage(int id)
 *    回调id的二维码初始配置坐标信息
 */
CPointsFourWorld DetctQrcode::getInitMarkMessage(const int id)
{
    return fixed_mark_world_;
}

//返回原始像素点坐标
vector<CPointsFour> DetctQrcode::detectMarks(cv::Mat image, int &MarkNum, const bool Is_global=false )
{
    vector<int> id_detected;
    vector< vector<CPointsFour> > coners_VVector;

    if (undistort_ && Is_global)          //去畸变    #define undistort 1
        cv::remap(image, image, map1_, map2_, cv::INTER_LINEAR);

    cv::Mat gray = image.clone();
    cv::cvtColor(image, gray,CV_BGR2GRAY);
    show_landmark_img_ = image.clone();

    Point2i size(show_landmark_img_.cols, show_landmark_img_.rows);
    Point2i semi_size(show_landmark_img_.cols/2, show_landmark_img_.rows/2);

    cv::rectangle(show_landmark_img_,Point(semi_size.x-1, semi_size.y-1),Point(semi_size.x+1, semi_size.y+1),CV_RGB(0,255,0),1,8); //圈取图像中心点
    cv::line(show_landmark_img_,Point(semi_size.x, 0), Point(semi_size.x, size.y), CV_RGB(0,0,0), 1, 8);
    cv::line(show_landmark_img_,Point(0, semi_size.y), Point(size.x, semi_size.y), CV_RGB(0,0,0), 1, 8);

    TrackerSingleMarker  pTrackSingleMark(image.cols, image.rows, 5, 6, 6, 6, 0);

    pTrackSingleMark.setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
    if (!pTrackSingleMark.init(calibFile_.c_str(), 1.0f, 1000.0f)) // load MATLAB file
    {
        printf("ERROR:p_tracker_raw_marks_->init（） failed\n");
    }
    pTrackSingleMark.setPatternWidth(2.0);
    pTrackSingleMark.setBorderWidth(useBCH_ ? 0.125 : 0.25);    //  const bool useBCH = false;
    pTrackSingleMark.setUndistortionMode(ARToolKitPlus::UNDIST_LUT);
    pTrackSingleMark.setMarkerMode(useBCH_ ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);
    /// Turns automa.ic threshold calculation on/off
    pTrackSingleMark.setNumAutoThresholdRetries(1000);
    pTrackSingleMark.activateAutoThreshold(true);
    pTrackSingleMark.setNumAutoThresholdRetries(1000);

//    imshow("test",gray);
//    cv::waitKey(20);
    int tryCount = 50;
    while(tryCount < 253 )
    {
        tryCount = 300;
        int num_markers = 0;
        ARMarkerInfo* nMarker_info = NULL;

        std::vector<int> markerId = pTrackSingleMark.calc(gray.data, &nMarker_info, &num_markers);
        for (int i = 0 ; i < num_markers ; i++)
        {
            if ( (nMarker_info[i].id != -1) && (ht_[nMarker_info[i].id] > 0.1) )
            {
                ARMarkerInfo  Mark_info = nMarker_info[i];
                bool id_was_detected = false;
                for (int ii = 0 ; ii < id_detected.size(); ii++)
                {

                    if (Mark_info.id == id_detected[ii])
                    {

                        ConerPoint  center,corner0,corner1,corner2,corner3 ;
                        center.init(Mark_info.pos[0],Mark_info.pos[1]) ;
                        corner0.init(Mark_info.vertex[(4-Mark_info.dir+0)%4][0],Mark_info.vertex[(4-Mark_info.dir+0)%4][1]) ;
                        corner1.init(Mark_info.vertex[(4-Mark_info.dir+1)%4][0],Mark_info.vertex[(4-Mark_info.dir+1)%4][1]) ;
                        corner2.init(Mark_info.vertex[(4-Mark_info.dir+2)%4][0],Mark_info.vertex[(4-Mark_info.dir+2)%4][1]) ;
                        corner3.init(Mark_info.vertex[(4-Mark_info.dir+3)%4][0],Mark_info.vertex[(4-Mark_info.dir+3)%4][1]) ;

                        CPointsFour points_temp;
                        points_temp.id = Mark_info.id ;
                        points_temp.dir = Mark_info.dir;
                        points_temp.init(corner0,corner1,corner2,corner3,center);
                        coners_VVector[ii].push_back(points_temp);
                    }
                }
                if (id_was_detected == false)
                {                       // X_arr的填充顺序是nMarker_info[i]中i的顺序。。
                    id_detected.push_back(Mark_info.id);

                    ConerPoint  center,corner0,corner1,corner2,corner3 ;
                    center.init(Mark_info.pos[0],Mark_info.pos[1]) ;
                    corner0.init(Mark_info.vertex[(4-Mark_info.dir+0)%4][0],Mark_info.vertex[(4-Mark_info.dir+0)%4][1]) ;
                    corner1.init(Mark_info.vertex[(4-Mark_info.dir+1)%4][0],Mark_info.vertex[(4-Mark_info.dir+1)%4][1]) ;
                    corner2.init(Mark_info.vertex[(4-Mark_info.dir+2)%4][0],Mark_info.vertex[(4-Mark_info.dir+2)%4][1]) ;
                    corner3.init(Mark_info.vertex[(4-Mark_info.dir+3)%4][0],Mark_info.vertex[(4-Mark_info.dir+3)%4][1]) ;

                    CPointsFour points_temp;
                    points_temp.init(corner0,corner1,corner2,corner3,center);
                    points_temp.id = Mark_info.id;
                    vector<CPointsFour> coners_temp;
                    coners_temp.push_back(points_temp);
                    coners_VVector.push_back(coners_temp);
                }
            }
        }
    }//while  阈值


    vector<CPointsFour>  coners_aver;
    for (int i = 0 ; i < coners_VVector.size(); i++)
    {
        CPointsFour  c_temp;
        c_temp =  averageCPointsFour(coners_VVector[i]);
        coners_aver.push_back(c_temp);
    }

    MarkNum = coners_aver.size() ;
    drawCodes(show_landmark_img_, coners_aver);
    marks_convert_pub__->convertOnce(show_landmark_img_);  //display

    id_detected.clear();
    coners_VVector.clear();
    return coners_aver;
}

void DetctQrcode::drawCodes(Mat& img, const vector<CPointsFour> marks)   //draw the square-qrcode_fourSide
{
    for (int count = 0 ; count < marks.size(); count++)
    {
        CPointsFour  one_mark = marks.at(count);

        cv::line(img,cv::Point(one_mark.corn0.X, one_mark.corn0.Y),cv::Point(one_mark.corn1.X, one_mark.corn1.Y),CV_RGB(255,0,0),1,8);
        cv::line(img,cv::Point(one_mark.corn1.X, one_mark.corn1.Y),cv::Point(one_mark.corn2.X, one_mark.corn2.Y),CV_RGB(255,0,0),1,8);
        cv::line(img,cv::Point(one_mark.corn2.X, one_mark.corn2.Y),cv::Point(one_mark.corn3.X, one_mark.corn3.Y),CV_RGB(255,0,0),1,8);
        cv::line(img,cv::Point(one_mark.corn3.X, one_mark.corn3.Y),cv::Point(one_mark.corn0.X, one_mark.corn0.Y),CV_RGB(255,0,0),1,8);
        cv::rectangle(img,Point(one_mark.center.X-1, one_mark.center.Y-1),Point(one_mark.center.X+1, one_mark.center.Y+1),CV_RGB(0,255,0),1,8); //圈取图像中心点

        //string dir_str = int2str(one_mark.dir);
        string tx0 = "0";
        string tx1 = "1";
        string tx2 = "2";
        string tx3 = "3";


        cv::putText(img,tx0,cv::Point(one_mark.corn0.X, one_mark.corn0.Y),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
        cv::putText(img,tx1,cv::Point(one_mark.corn1.X, one_mark.corn1.Y),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
        cv::putText(img,tx2,cv::Point(one_mark.corn2.X, one_mark.corn2.Y),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
        cv::putText(img,tx3,cv::Point(one_mark.corn3.X, one_mark.corn3.Y),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));

        std::string text ="Id:"+ int2str(one_mark.id);
        cv::putText(img,text,Point(one_mark.center.X+80,one_mark.center.Y),CV_FONT_HERSHEY_COMPLEX,1,CV_RGB(0,0,255));
        // cv::putText(img,dir_str,cv::Point(400,400),CV_FONT_HERSHEY_COMPLEX,2,CV_RGB(0,0,255));
    }
}

std::string  DetctQrcode::int2str(int num)
{
    std::stringstream ss;
    ss << num;
    std::string text = ss.str();
    return text;
}

void DetctQrcode::createMap( MapType &inputData, char * filename)
{
    std::string line;
    std::string startBraceEnd(">");
    std::string endBraceStart("</");
    std::ifstream infile(filename);
    while (std::getline(infile, line))
    {
        bool readFine = false;
        if (line.size() > 0 && line[0] == '<' && line[line.size()-1] == '>' )
        {
            std::size_t pSBE = line.find(startBraceEnd);
            std::size_t pEBS = line.find(endBraceStart);
            if ( pSBE!=std::string::npos && pEBS!=std::string::npos && pSBE < pEBS)
            {
                std::string marker1 = line.substr(1,pSBE-1);
                std::string marker2 = line.substr(pEBS+2,line.size()-pEBS-3);
                if ( marker1 == marker2 )
                {
                    std::string content = line.substr(pSBE+1,pEBS-pSBE-1);
                    inputData.insert( pair<string, string>(marker1, content) );
                    readFine = true;
                }
            }
        }
        if ( readFine == true )
        {
            cout<<line<<" READ FINE "<<endl;
        }
        else
        {
            cout<<line<<" ERROR ERROR ERROR !!!!!!!!!!!!! "<<endl;
        }
    }
}
void DetctQrcode::undistortImage( IplImage* frame, cv::Mat & map1,cv::Mat & map2)
{
    cout << "frame: " << frame->width << " x " << frame->height << endl;
    cv::Mat viewDistorted(frame,false);
    cv::Mat viewRectified;
    // warps the image using the precomputed maps.
    cv::remap(viewDistorted, viewRectified, map1, map2, cv::INTER_LINEAR);
    cv::Mat FM(frame);
    viewRectified.copyTo(FM);
}
std::string DetctQrcode::arrToStr(const char * c, int i )
{
    std::stringstream ss;
    ss << c << i ;
    std::string res;
    ss >> res;
    return res;
}
void   DetctQrcode::gradientNormalizePic(cv::Mat &cutout)
{
    double rowsDiff = 0;
    for ( int x = 0; x < cutout.cols; x++ )
    {
        int diffCounter = 0;
        int diffAcc = 0;
        for ( int y = 1; y < cutout.rows; y++ )
        {
            int diff = cutout.at<uchar>(y,x) - cutout.at<uchar>(y-1,x) ;
            if ( abs(diff) < 5 )
            {
                diffAcc += diff;
                diffCounter++;
            }
        }
        rowsDiff += (double) diffAcc / (double) diffCounter;
    }

    rowsDiff = (double) rowsDiff / (double) cutout.cols;


    double colsDiff = 0;
    for ( int y = 0; y < cutout.rows; y++ )
    {
        int diffCounter = 0;
        int diffAcc = 0;
        for ( int x = 1; x < cutout.cols; x++ )
        {
            int diff = cutout.at<uchar>(y,x) - cutout.at<uchar>(y,x-1) ;
            if ( abs(diff) < 5 )
            {
                diffAcc += diff;
                diffCounter++;
            }
        }
        colsDiff += (double) diffAcc / (double) diffCounter;
    }
    colsDiff = (double) colsDiff / (double) cutout.rows ;
    // 均值行列差分作为起始项
    double startdr = rowsDiff * cutout.rows / 2.0 ;
    double startdc = colsDiff * cutout.cols / 2.0 ;

    for ( int x = 0; x < cutout.cols; x++ )
    {
        double adjr = startdr;
        for ( int y = 0; y < cutout.rows; y++ )
        {
            int newVal = ( (double) cutout.at<uchar>(y,x) ) + adjr;
            newVal = newVal >= 0  ? newVal : 0;
            newVal = newVal < 256 ? newVal :255;
            cutout.at<uchar>(y,x) = newVal;
            adjr -= rowsDiff*5;
        }
    }

    for ( int y = 0; y < cutout.rows; y++ )
    {
        double adjc = startdc;
        for ( int x = 1; x < cutout.cols; x++ )
        {
            int newVal = ( (double) cutout.at<uchar>(y,x) ) + adjc;
            newVal = newVal >= 0  ? newVal : 0;
            newVal = newVal < 256 ? newVal :255;
            cutout.at<uchar>(y,x) = newVal;
            adjc -= colsDiff*5;

        }
    }
}

double DetctQrcode::averageVector(vector<double> data, double multiplier, double shift, double range ,int bucketAmount)
{
    int count = 0;
    double accum = 0.0;
    for (int i = 0 ; i < data.size();i++)
    {
        count++;
        accum = accum + data[i];
    }
    return  accum / count ;
}

/*边长计算  矩形四个顶点  */
double DetctQrcode::sideCalc(ARMarkerInfo Mark)
{
    double ddAcc = 0.0;
    double xd,yd,dd;
    for ( int i = 0 ; i < 4 ; i++ )
    {
        xd = Mark.vertex[i][0] - Mark.vertex[(i+1)%4][0];
        yd = Mark.vertex[i][1] - Mark.vertex[(i+1)%4][1];
        dd = sqrt(xd*xd + yd*yd);
        ddAcc = ddAcc + dd;
    }
    return ddAcc / 4.0;
}

CPointsFour DetctQrcode::averageCPointsFour(vector<CPointsFour> data)
{
    int count = 0;
    int id = -1;
    int dir = -1;
    ConerPoint  center, corner0, corner1, corner2, corner3 ;
    center.init(0,0); corner0.init(0,0); corner1.init(0,0); corner2.init(0,0); corner3.init(0,0);
    for (int i = 0 ; i < data.size();i++)
    {
        count++;
        id = data[i].id;
        dir = data[i].dir;
        center.X += data[i].center.X ;
        center.Y += data[i].center.Y ;
        corner0.X += data[i].corn0.X ;
        corner0.Y += data[i].corn0.Y ;
        corner1.X += data[i].corn1.X ;
        corner1.Y += data[i].corn1.Y ;
        corner2.X += data[i].corn2.X ;
        corner2.Y += data[i].corn2.Y ;
        corner3.X += data[i].corn3.X ;
        corner3.Y += data[i].corn3.Y ;
    }
    center.X =    center.X / count ;
    center.Y =    center.Y / count ;
    corner0.X =  corner0.X / count ;
    corner0.Y =  corner0.Y / count ;
    corner1.X =  corner1.X / count ;
    corner1.Y =  corner1.Y / count ;
    corner2.X =  corner2.X / count ;
    corner2.Y =  corner2.Y / count ;
    corner3.X =  corner3.X / count ;
    corner3.Y =  corner3.Y / count ;
    CPointsFour points_temp;
    points_temp.init(corner0,corner1,corner2,corner3,center);
    points_temp.id = id;
    return points_temp;
}

bool DetctQrcode::codeTestProcess(Mat image_raw )
{
    bool flag = false;
    cv::Mat image_gray = image_raw.clone();
    cvtColor(image_raw, image_gray,CV_BGR2GRAY);//灰度化

    int width = image_gray.cols;
    int height = image_gray.rows;
    int threshold;

    cv::Mat image_gray_threshold = image_gray.clone();
    cv::Mat image_gray_gradient;

    double sec = -1;
    struct timeval tim1;
    struct timeval tim2;
    gettimeofday(&tim1, NULL);
    equalizeHist(image_gray_threshold,image_gray_gradient);  // 直方图均衡化.
    gettimeofday(&tim2, NULL);
    sec = (tim2.tv_sec - tim1.tv_sec)*1000000.0 + (tim2.tv_usec - tim1.tv_usec);// 微秒
    cout<<" equalizeHist time "<<sec<<"  us "<<endl;

    //gradientNormalizePic(image_gray_gradient);  // 梯度补偿

    //图像显示与保存
    char threshold_img_name[100];
    sprintf(threshold_img_name,"%s%d%s", "./conpensate/raw",threshold, ".jpg");//保存的图片名
    //       imshow("image_gray_threshold",image_gray_threshold);
    //        cv::imwrite(threshold_img_name,image_gray_threshold);

    char conpensate_img_name[100];
    sprintf(conpensate_img_name,"%s%d%s", "./conpensate/",threshold, ".jpg");//保存的图片名
    //        imshow("image_gray_gradient",image_gray_gradient);
    //        cv::imwrite(conpensate_img_name,image_gray_gradient);

    //--------------------------------------------------------------------------------------------------
    TrackerSingleMarker* ptrackerWhole = new TrackerSingleMarker(width, height, 5, 6, 6, 6, 0);

    ptrackerWhole->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
    // load a camera file.
    if (! ptrackerWhole->init(calibFile_.c_str(), 1.0f, 1000.0f)) // load MATLAB file
    {
        printf("ERROR: init() failed\n");
    }
    ptrackerWhole->setPatternWidth(2.0);
    ptrackerWhole->setBorderWidth(useBCH_ ? 0.125 : 0.25);    //  const bool useBCH = false;
    ptrackerWhole->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);
    ptrackerWhole->setMarkerMode(useBCH_ ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);
    ptrackerWhole->setNumAutoThresholdRetries(1000);
    ptrackerWhole->activateAutoThreshold(true);/// Turns automatic threshold calculation on/off
    ptrackerWhole->setNumAutoThresholdRetries(1000);

    ARMarkerInfo* ar_compensate_info;
    ARMarkerInfo* ar_threshold_info;
    int ar_compensate_num = 0;
    int ar_threshold_num = 0;

    double sec1 = -1;
    timeb time11, time21;
    ftime(&time11);
    std::vector<int> ar_threshold_id = ptrackerWhole->calc(image_gray_threshold.data, &ar_threshold_info, &ar_threshold_num);
    ftime(&time21);
    sec1 = (time21.time-time11.time)*1000.0 + (time21.millitm-time11.millitm);
    cout<<"  "<<sec1<<"  "<<endl;

    std::vector<int> ar_compensate_id = ptrackerWhole->calc(image_gray_gradient.data, &ar_compensate_info, &ar_compensate_num);

    //显示检测结果  阈值下  // 补偿下
    for(int i = 0 ; i < ar_threshold_num ; i++)
    {
        if(ar_threshold_info[i].id != -1)
        {
            flag = true;
            ARMarkerInfo  Mark_info = ar_threshold_info[i];
            // threshold id cf width height area dir pos vertex line
            fThreshold_img_process<<" "<<threshold<<" "<<Mark_info.id<<" "<<Mark_info.cf<<" "<<width<<" "<<height<<" "<<Mark_info.area<<" "<<Mark_info.dir<<" "
                                 <<" "<<Mark_info.pos[0]<<" "<<Mark_info.pos[1]
                                <<" "<<Mark_info.vertex[(4-Mark_info.dir+0)%4][0]<<" "<<Mark_info.vertex[(4-Mark_info.dir+0)%4][1]
                    <<" "<<Mark_info.vertex[(4-Mark_info.dir+1)%4][0]<<" "<<Mark_info.vertex[(4-Mark_info.dir+1)%4][1]
                    <<" "<<Mark_info.vertex[(4-Mark_info.dir+2)%4][0]<<" "<<Mark_info.vertex[(4-Mark_info.dir+2)%4][1]
                    <<" "<<Mark_info.vertex[(4-Mark_info.dir+3)%4][0]<<" "<<Mark_info.vertex[(4-Mark_info.dir+3)%4][1]

                    <<" "<<Mark_info.line[(4-Mark_info.dir+0)%4][0]<<" "<<Mark_info.line[(4-Mark_info.dir+0)%4][1]<<" "<<Mark_info.line[(4-Mark_info.dir+0)%4][2]
                    <<" "<<Mark_info.line[(4-Mark_info.dir+1)%4][0]<<" "<<Mark_info.line[(4-Mark_info.dir+1)%4][1]<<" "<<Mark_info.line[(4-Mark_info.dir+2)%4][2]
                    <<" "<<Mark_info.line[(4-Mark_info.dir+2)%4][0]<<" "<<Mark_info.line[(4-Mark_info.dir+2)%4][1]<<" "<<Mark_info.line[(4-Mark_info.dir+3)%4][2]
                    <<" "<<Mark_info.line[(4-Mark_info.dir+3)%4][0]<<" "<<Mark_info.line[(4-Mark_info.dir+3)%4][1]<<" "<<Mark_info.line[(4-Mark_info.dir+4)%4][2]
                    <<endl;
        }
    }

    for(int i = 0 ; i < ar_compensate_num ; i++)
    {
        if(ar_compensate_info[i].id != -1)
        {
            ARMarkerInfo  Mark_info = ar_compensate_info[i];
            // threshold id cf area dir pos vertex line
            fConpensate_img_process<<" "<<threshold<<" "<<Mark_info.id<<" "<<Mark_info.cf<<" "<<width<<" "<<height<<" "<<Mark_info.area<<" "<<Mark_info.dir<<" "
                                  <<" "<<Mark_info.pos[0]<<" "<<Mark_info.pos[1]
                                 <<" "<<Mark_info.vertex[(4-Mark_info.dir+0)%4][0]<<" "<<Mark_info.vertex[(4-Mark_info.dir+0)%4][1]
                    <<" "<<Mark_info.vertex[(4-Mark_info.dir+1)%4][0]<<" "<<Mark_info.vertex[(4-Mark_info.dir+1)%4][1]
                    <<" "<<Mark_info.vertex[(4-Mark_info.dir+2)%4][0]<<" "<<Mark_info.vertex[(4-Mark_info.dir+2)%4][1]
                    <<" "<<Mark_info.vertex[(4-Mark_info.dir+3)%4][0]<<" "<<Mark_info.vertex[(4-Mark_info.dir+3)%4][1]

                    <<" "<<Mark_info.line[(4-Mark_info.dir+0)%4][0]<<" "<<Mark_info.line[(4-Mark_info.dir+0)%4][1]<<" "<<Mark_info.line[(4-Mark_info.dir+0)%4][2]
                    <<" "<<Mark_info.line[(4-Mark_info.dir+1)%4][0]<<" "<<Mark_info.line[(4-Mark_info.dir+1)%4][1]<<" "<<Mark_info.line[(4-Mark_info.dir+2)%4][2]
                    <<" "<<Mark_info.line[(4-Mark_info.dir+2)%4][0]<<" "<<Mark_info.line[(4-Mark_info.dir+2)%4][1]<<" "<<Mark_info.line[(4-Mark_info.dir+3)%4][2]
                    <<" "<<Mark_info.line[(4-Mark_info.dir+3)%4][0]<<" "<<Mark_info.line[(4-Mark_info.dir+3)%4][1]<<" "<<Mark_info.line[(4-Mark_info.dir+4)%4][2]
                    <<endl;
        }
    }

    return flag;
}


//===========================  像素/尺寸 转换 =============================
void DetctQrcode::imgCornerToWorld(CPointsFour &point_four)
{
    point_four.corn0 = imTotruePos( point_four.corn0.X, point_four.corn0.Y,point_four.id);
    point_four.corn1 = imTotruePos( point_four.corn1.X, point_four.corn1.Y,point_four.id);
    point_four.corn2 = imTotruePos( point_four.corn2.X, point_four.corn2.Y,point_four.id);
    point_four.corn3 = imTotruePos( point_four.corn3.X, point_four.corn3.Y,point_four.id);
    point_four.center = imTotruePos(point_four.center.X,point_four.center.Y,point_four.id);
}

/**
 * @brief DetctQrcode::imCornerToWorld
 * 将图像提取的像素坐标值转换成实际的物理距离值
 * 相应数值的显示
 * @param point_four   五点集
 */
void DetctQrcode::imgCornerToWorld(CPointsFour& dst,const CPointsFour src)
{
    dst.corn0 = imTotruePos( src.corn0.X, src.corn0.Y,src.id);
    dst.corn1 = imTotruePos( src.corn1.X, src.corn1.Y,src.id);
    dst.corn2 = imTotruePos( src.corn2.X, src.corn2.Y,src.id);
    dst.corn3 = imTotruePos( src.corn3.X, src.corn3.Y,src.id);
    dst.center = imTotruePos(src.center.X,src.center.Y,src.id);
    dst.id = src.id;
}
//旋转是相对
/**
 * @brief DetctQrcode::imTotruePos
 * 根据几何成像模型把图像像素值转换成物理值
 * @param width     宽(y向)--> x
 * @param height    高（x向）--> y
 * @param id
 */
ConerPoint DetctQrcode::imTotruePos(const double width, const double height,const int id)
{
    double centXoff = height - camInnerPara_.dy;
    double centYoff = camInnerPara_.dx - width ;          //采取内参校正数值

    double x_cm  = centXoff * ht_[id] / camInnerPara_.fy; //camInnerFocus;
    double y_cm =  centYoff * ht_[id] / camInnerPara_.fx; //camInnerFocus;

    ConerPoint cornPoint;
    cornPoint.init(x_cm,y_cm);
    return  cornPoint;
}



/**
 * @brief DetctQrcode::imgToWorld
 * 将图像提取的像素坐标值转换成实际的物理距离值  --->利用已知二维码的边信息
 * @param point_four   五点集
 */
void DetctQrcode::pixToMeter(CPointsFour& dst,CPointsFour& src)
{
    float side[4];
    float side0x = (src.corn0.X - src.corn1.X)*(src.corn0.X - src.corn1.X);
    float side0y = (src.corn0.Y - src.corn1.Y)*(src.corn0.Y - src.corn1.Y);
    side[0]  = sqrt(side0x + side0y);

    float side1x = (src.corn1.X - src.corn2.X)*(src.corn1.X - src.corn2.X);
    float side1y = (src.corn1.Y - src.corn2.Y)*(src.corn1.Y - src.corn2.Y);
    side[1]  = sqrt(side1x + side1y);

    float side2x = (src.corn2.X - src.corn3.X)*(src.corn2.X - src.corn3.X);
    float side2y = (src.corn2.Y - src.corn3.Y)*(src.corn2.Y - src.corn3.Y);
    side[2]  = sqrt(side2x + side2y);

    float side3x = (src.corn3.X - src.corn0.X)*(src.corn3.X - src.corn0.X);
    float side3y = (src.corn3.Y - src.corn0.Y)*(src.corn3.Y - src.corn0.Y);
    side[3]  = sqrt(side3x + side3y);

    float scale[4];
    scaleFromSide(scale,side,src.id);
    float scale_center = (scale[0] + scale[1] + scale[2] + scale[3])/4;

    dst.corn0 = pixToMeterFromScale( src.corn0.X, src.corn0.Y,scale[0]);
    dst.corn1 = pixToMeterFromScale( src.corn1.X, src.corn1.Y,scale[1]);
    dst.corn2 = pixToMeterFromScale( src.corn2.X, src.corn2.Y,scale[2]);
    dst.corn3 = pixToMeterFromScale( src.corn3.X, src.corn3.Y,scale[3]);
    dst.center = pixToMeterFromScale(src.center.X,src.center.Y,scale_center);
    dst.id = src.id;
}

void DetctQrcode::scaleFromSide(float scale[],float side[],int id)
{
    scale[0] = sqrt( ((side_sizes_[id]/side[0])*(side_sizes_[id]/side[0]) +
            (side_sizes_[id]/side[3])*(side_sizes_[id]/side[3]))/2.0f   );

    scale[1] = sqrt( ((side_sizes_[id]/side[0])*(side_sizes_[id]/side[0]) +
            (side_sizes_[id]/side[1])*(side_sizes_[id]/side[1]))/2.0f   );

    scale[2] = sqrt( ((side_sizes_[id]/side[2])*(side_sizes_[id]/side[2]) +
            (side_sizes_[id]/side[1])*(side_sizes_[id]/side[1]))/2.0f   );

    scale[3] = sqrt( ((side_sizes_[id]/side[2])*(side_sizes_[id]/side[2]) +
            (side_sizes_[id]/side[3])*(side_sizes_[id]/side[3]))/2.0f   );
}

ConerPoint DetctQrcode::pixToMeterFromScale(double width, double height, float scale)
{
    double centXoff = height - camInnerPara_.dy;
    double centYoff = camInnerPara_.dx - width ;          //采取内参校正数值

    double x_cm  = centXoff * scale; //camInnerFocus;
    double y_cm =  centYoff * scale; //camInnerFocus;

    ConerPoint cornPoint;
    cornPoint.init(x_cm,y_cm);
    return  cornPoint;
}

Point2f DetctQrcode::trueToimPos(Point2f pos_world,int id)
{
    double centUoff = pos_world.x * camInnerPara_.fy / ht_[id];  //camInnerFocus;
    double centVoff = pos_world.y * camInnerPara_.fx / ht_[id] ; //camInnerFocus;

    double u_pix = camInnerPara_.dy + centUoff;
    double v_pix = camInnerPara_.dx - centVoff ;          //采取内参校正数值

    return  Point2f(v_pix,u_pix);
}

//返回原始像素点坐标
vector<CPointsFour> DetctQrcode::detectMarksImprove(cv::Mat image, int &MarkNum, Point3f& search_time, const bool Is_global=false )
{
    static int total_search = 0;
    static int raw_search = 0;
    static int compensate_search = 0;

    vector<int> id_detected;
    vector< vector<CPointsFour> > coners_VVector;

    if (undistort_ && Is_global)          //去畸变    #define undistort 1
        cv::remap(image, image, map1_, map2_, cv::INTER_LINEAR);

    cv::Mat gray = image.clone();
    cv::cvtColor(image,gray,CV_BGR2GRAY);
    show_landmark_img_ = image.clone();

    Point2i size(show_landmark_img_.cols, show_landmark_img_.rows);
    Point2i semi_size(show_landmark_img_.cols/2, show_landmark_img_.rows/2);

    cv::rectangle(show_landmark_img_,Point(semi_size.x-1, semi_size.y-1),Point(semi_size.x+1, semi_size.y+1),CV_RGB(0,255,0),1,8); //圈取图像中心点
    cv::line(show_landmark_img_,Point(semi_size.x, 0), Point(semi_size.x, size.y), CV_RGB(0,0,0), 1, 8);
    cv::line(show_landmark_img_,Point(0, semi_size.y), Point(size.x, semi_size.y), CV_RGB(0,0,0), 1, 8);

    TrackerSingleMarker* pTrackSingleMark = new TrackerSingleMarker(image.cols, image.rows, 5, 6, 6, 6, 0);

    pTrackSingleMark->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);
    //    if (!p_tracker_raw_marks_->init("./data/no_distortion.cal", 1.0f, 1000.0f)) // load MATLAB file
    if (!pTrackSingleMark->init(calibFile_.c_str(), 1.0f, 1000.0f)) // load MATLAB file
    {
        printf("ERROR:p_tracker_raw_marks_->init（） failed\n");
    }
    pTrackSingleMark->setPatternWidth(2.0);
    pTrackSingleMark->setBorderWidth(useBCH_ ? 0.125 : 0.25);    //  const bool useBCH = false;
    pTrackSingleMark->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);
    pTrackSingleMark->setMarkerMode(useBCH_ ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);
    /// Turns automatic threshold calculation on/off
    pTrackSingleMark->activateAutoThreshold(true);
    pTrackSingleMark->setNumAutoThresholdRetries(1000);

    int tryCount = 50;
    int Is_compensated = 0;
    bool Is_found = false;
    while(Is_compensated<2 && !Is_found)
    {
        if(Is_compensated == 1)
        {//图像补偿
            equalizeHist(gray,gray);  // 直方图均衡化.
        }
        while(tryCount < 180 && !Is_found )
        {
            total_search++;
            //tryCount = 300;
            int num_markers = 0;
            ARMarkerInfo* nMarker_info = NULL;
            std::vector<int> markerId = pTrackSingleMark->calc(gray.data, &nMarker_info, &num_markers);
            for (int i = 0 ; i < num_markers ; i++)
            {
                if ( (nMarker_info[i].id != -1) && (ht_[nMarker_info[i].id] > 0.1) )
                {
                    Is_found = true;
                    ARMarkerInfo  Mark_info = nMarker_info[i];
                    bool id_was_detected = false;
                    for (int ii = 0 ; ii < id_detected.size(); ii++)
                    {
                        if (Mark_info.id == id_detected[ii])
                        {
                            ConerPoint  center,corner0,corner1,corner2,corner3 ;
                            center.init(Mark_info.pos[0],Mark_info.pos[1]) ;
                            corner0.init(Mark_info.vertex[(4-Mark_info.dir+0)%4][0],Mark_info.vertex[(4-Mark_info.dir+0)%4][1]) ;
                            corner1.init(Mark_info.vertex[(4-Mark_info.dir+1)%4][0],Mark_info.vertex[(4-Mark_info.dir+1)%4][1]) ;
                            corner2.init(Mark_info.vertex[(4-Mark_info.dir+2)%4][0],Mark_info.vertex[(4-Mark_info.dir+2)%4][1]) ;
                            corner3.init(Mark_info.vertex[(4-Mark_info.dir+3)%4][0],Mark_info.vertex[(4-Mark_info.dir+3)%4][1]) ;

                            CPointsFour points_temp;
                            points_temp.id = Mark_info.id ;
                            points_temp.dir = Mark_info.dir;
                            points_temp.init(corner0,corner1,corner2,corner3,center);
                            coners_VVector[ii].push_back(points_temp);
                        }
                    }
                    if (id_was_detected == false)
                    {   // X_arr的填充顺序是nMarker_info[i]中i的顺序。。
                        id_detected.push_back(Mark_info.id);
                        ConerPoint  center,corner0,corner1,corner2,corner3 ;
                        center.init(Mark_info.pos[0],Mark_info.pos[1]) ;
                        corner0.init(Mark_info.vertex[(4-Mark_info.dir+0)%4][0],Mark_info.vertex[(4-Mark_info.dir+0)%4][1]) ;
                        corner1.init(Mark_info.vertex[(4-Mark_info.dir+1)%4][0],Mark_info.vertex[(4-Mark_info.dir+1)%4][1]) ;
                        corner2.init(Mark_info.vertex[(4-Mark_info.dir+2)%4][0],Mark_info.vertex[(4-Mark_info.dir+2)%4][1]) ;
                        corner3.init(Mark_info.vertex[(4-Mark_info.dir+3)%4][0],Mark_info.vertex[(4-Mark_info.dir+3)%4][1]) ;

                        CPointsFour points_temp;
                        points_temp.init(corner0,corner1,corner2,corner3,center);
                        points_temp.id = Mark_info.id;
                        vector<CPointsFour> coners_temp;
                        coners_temp.push_back(points_temp);
                        coners_VVector.push_back(coners_temp);
                    }
                }
            }
            delete nMarker_info;
            tryCount += 5;
        }//while  阈值
        if(Is_compensated == 1 && Is_found)
            compensate_search++;
        if(Is_compensated == 0 && Is_found)
             raw_search++;
        Is_compensated ++;
    }//while 补偿
    cout<<"search: total raw compensate"<< total_search <<"  "<< raw_search <<"  "<< compensate_search<<"   "<<endl;
    search_time.x = total_search;
    search_time.y = raw_search;
    search_time.z = compensate_search;

    vector<CPointsFour>  coners_aver;
    for (int i = 0 ; i < coners_VVector.size(); i++)
    {
        CPointsFour  c_temp;
        c_temp =  averageCPointsFour(coners_VVector[i]);
        coners_aver.push_back(c_temp);
    }
    MarkNum = coners_aver.size() ;

    drawCodes(show_landmark_img_, coners_aver);
    marks_convert_pub__->convertOnce(show_landmark_img_);  //display

    delete pTrackSingleMark;
    id_detected.clear();
    coners_VVector.clear();
    return coners_aver;
}
