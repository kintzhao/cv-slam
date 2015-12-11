/*  说明： 文件路径参数
 * argv[1] : 相机校正相关的数据    ./data/config_small.in
 * argv[2] : ekfslam相关的参数    ./data/DataConfig.xml
 * 使用时注意 相机镜头的选用（大小），二维码id的选用。
*/
#include <ros/ros.h>
#include "./class/detectqrcode.h"
#include "./graph_slam.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "qr_slam");
    graphSlam graph_slam(argv[1],argv[2]);
    ros::spin();

    return 0;
}
