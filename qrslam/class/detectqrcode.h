#ifndef QRCODELOCALIZER_H
#define QRCODELOCALIZER_H

#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/timeb.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <math.h>
#include <iostream>
#include <vector>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>

//#include "../../include/ARToolKitPlus/TrackerSingleMarker.h"
//#include "../../include/ARToolKitPlus/ar.h"

#include "../../ARToolkitPlus/include/ARToolKitPlus/TrackerSingleMarker.h"
#include "../../ARToolkitPlus/include/ARToolKitPlus/ar.h"

#include "../../image_convert/image_converter.h"

#include <sys/timeb.h>
#include <sys/time.h>

using ARToolKitPlus::TrackerSingleMarker;
using ARToolKitPlus::ARMarkerInfo;
using namespace std;
using namespace cv;

/// struct
typedef struct ConerPoint
{
    double X;
    double Y;
    void init(double x1,double y1)
    {
        X= x1;
        Y= y1;
    }
    void init(ConerPoint x1)
    {
        X=x1.X ;
        Y=x1.Y ;
    }
    void addOffset(double u, double v)
    {
       X +=u;
       Y +=v;
    }
}ConerPoint;

typedef struct CPointsFour
{
    int id;
    int dir;
    ConerPoint corn0;
    ConerPoint corn1;
    ConerPoint corn2;
    ConerPoint corn3;
    ConerPoint center;

    void init(ConerPoint p0,ConerPoint p1,ConerPoint p2,ConerPoint p3,ConerPoint p4)
    {
        corn0.init(p0);
        corn1.init(p1);
        corn2.init(p2);
        corn3.init(p3);
        center.init(p4);
    }
    void addOffset(double u, double v)
    {
      corn0.addOffset(u,v);
      corn1.addOffset(u,v);
      corn2.addOffset(u,v);
      corn3.addOffset(u,v);
      center.addOffset(u,v);
    }

}CPointsFour;


typedef struct ConerPointWorld
{
    float X;
    float Y;
    float Z;
    void init(float x1,float y1,float z1)
    {
        X= x1;
        Y= y1;
        Z= z1;
    }
    void init(ConerPointWorld x1)
    {
        X=x1.X ;
        Y=x1.Y ;
        Z=x1.Z ;
    }
}ConerPointWorld;

typedef struct CPointsFourWorld
{
    int ID;
    ConerPointWorld corn0;
    ConerPointWorld corn1;
    ConerPointWorld corn2;
    ConerPointWorld corn3;
    ConerPointWorld center;

    void init(ConerPointWorld p0,ConerPointWorld p1,ConerPointWorld p2,ConerPointWorld p3,ConerPointWorld p4)
    {
        corn0.init(p0);
        corn1.init(p1);
        corn2.init(p2);
        corn3.init(p3);
        center.init(p4);
    }

}CPointsFourWorld;

/// struct
typedef struct Pose3D
{
    double X;
    double Y;
    double Theta;
}Pose3D;

typedef struct CamraInner
{
    double fx;
    double fy;
    double dx;
    double dy;
}CamraInner;

typedef struct QrLandMark
{
int    Id   ;
int    Dir  ;
float  Theta ;
double X;
double Y;
double Side;
}QrLandMark;

typedef std::map <std::string, std::string> MapType;

class DetctQrcode
{
public:
    /// construction & destruction
    DetctQrcode(char * mapFile, int id_fixed_mark);
    ~DetctQrcode();

    int img_W_ = 640 ;
    int img_H_ = 480 ;


public:
    /// public method
    vector<CPointsFour> detectLandmarks(cv::Mat image, int &MarkNum) ;
    vector<CPointsFour> detectMarks(cv::Mat image, int &MarkNum, const bool Is_global) ;

    ConerPoint  imTotruePos(const double width, const double height, const int id);
    std::string  int2str(int num);
    CPointsFourWorld getInitMarkMessage(const int id);
    CPointsFourWorld fixed_mark_world_;

private:
    /// private method
    std::string arrToStr( const char *c, int i );
    void   createMap( MapType &input_data_, char * filename);
    void   undistortImage( IplImage* frame, cv::Mat & map1_,cv::Mat & map2_);
    void   gradientNormalizePic(cv::Mat &cutout);

    double averageVector(vector<double> data, double multiplier, double shift, double range ,int bucketAmount);
    double sideCalc(ARMarkerInfo Mark_);

private:
    /// private member
    int undistort_;//   图像校正关闭0 / 1开
    std::string    calibFile_;
    CamraInner camInnerPara_;
    vector<float>   ht_;
    vector<int>     orients_;
    vector<int>     side_sizes_;
    cv::Point2i     middle_;

    MapType         input_data_;
    bool            useBCH_ ;

private:

    cv::Mat camera_matrix_;
    cv::Mat distcoeffs_ ;
    cv::Mat map1_,map2_;

public:

    cv::Mat show_landmark_img_;
    ImageConverter* marks_convert_pub__ = NULL;

    CPointsFour averageCPointsFour(vector<CPointsFour> data);
    void  imgCornerToWorld(CPointsFour &point_four) ;
    void  imgCornerToWorld(CPointsFour& dst, const CPointsFour src);
    void  pixToMeter(CPointsFour& dst,CPointsFour& src);
    void scaleFromSide(float scale[],float side[],int id);
    ConerPoint pixToMeterFromScale(double width,double height,float scale);
    bool readFixedMarkInfo(int id) ;
    bool readConfigureInfo(int max_num);
    bool codeTestProcess(cv::Mat image_raw);
    Point2f trueToimPos(Point2f pos_world, int id);

    void drawCodes(Mat& img, const vector<CPointsFour> marks) ;
    vector<CPointsFour> detectMarksImprove(cv::Mat image, int &MarkNum, Point3f &search_time, const bool Is_global);
    //TrackerSingleMarker* pTrackSingleMark ;

};

#endif // QRCODELOCALIZER_H
