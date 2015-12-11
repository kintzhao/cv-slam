/*  说明： 文件路径参数
 * argv[1] : 相机校正相关的数据    ./data/config_small.in
 * argv[2] : ekfslam相关的参数    ./data/DataConfig.xml
 * 使用时注意 相机镜头的选用（大小），二维码id的选用。
*/
#include <ros/ros.h>
#include "./class/detectqrcode.h"
#include "./qrslam.h"
#include <stdlib.h>

#define SELECT_ROBOT_POS_AS_ORIGIN  0 // 0 表示20 mark 为原点
//ofstream fmain("main.txt");
using namespace cv;
using namespace std;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "qr_slam");
    CVSlam qr_slam(argv[1],argv[2]);
    ros::spin();
    return 0;
}





//生成配置文件
//#include <cstring>
//#include <cstdio>
//#include <math.h>
//#include <iostream>
//#include <vector>
//#include <map>
//#include <string>
//#include <fstream>
//using  namespace std;
//ofstream fmain("main.txt");

//int main(int argc, char** argv)
//{
//    for(int i=1;i<=20;i++)
//    {
//      fmain<<"<ht"<<i<<">"<<2.295<<"</ht"<<i<<">"<<endl;
//      fmain<<"<orients"<<i<<">"<<2.295<<"</orients"<<i<<">"<<endl;
//      fmain<<"<sideSizes"<<i<<">"<<2.295<<"</sideSizes"<<i<<">"<<endl;
//    }
////    for(int i=1;i<=4;i++)
////    {
////        int count=1;
////        for(int j=0;i<5;i++)
////        {
////            count += 1.5*j;
////         cout<<"<sideSizes"<<i<<">"<<count<<"</sideSizes"<<i<<">"<<endl;
////        }
////    }


//    return 0;
//}


