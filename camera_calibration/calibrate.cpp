/*------------------------------------------------------------------------------------------*\
   This file contains material supporting chapter 9 of the cookbook:  
   Computer Vision Programming using the OpenCV Library. 
   by Robert Laganiere, Packt Publishing, 2011.

   This program is free software; permission is hereby granted to use, copy, modify, 
   and distribute this source code, or portions thereof, for any purpose, without fee, 
   subject to the restriction that the copyright notice may not be removed 
   or altered from any source or altered source distribution. 
   The software is released on an as-is basis and without any warranties of any kind. 
   In particular, the software is not guaranteed to be fault-tolerant or free from failure. 
   The author disclaims all warranties with regard to this software, any use, 
   and any consequent failure, is purely the responsibility of the user.
 
   Copyright (C) 2010-2011 Robert Laganiere, www.laganiere.name
\*------------------------------------------------------------------------------------------*/

#include <iostream>
#include <iomanip>       //std::setw(2) << std::setfill('0') << i << ".jpg";//输出0i.jpg
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/opencv.hpp"

#include "./class/CameraCalibrator.h"
#include "./class/CameraCalibrator.h"

using namespace cv;
int main()
{
	cv::namedWindow("Image");
	cv::Mat image;
	std::vector<std::string> filelist;
    // Notion the  file dirctory
    FileStorage camcalibrate("./data/camcalibrate_small_opencv.xml", FileStorage::WRITE);
    camcalibrate << "read" << "camera calibration by kint at 15.02.03";
    camcalibrate << "imageCount" << 12;
	// generate list of chessboard image filename
    for (int i=1; i<=24; i++)
    {
		std::stringstream str;
        //frame_001.png
        str << "./chessboards/" <<"frame_0"<< std::setw(2) << std::setfill('0') << i << ".png"; //输出0i.jpg
		std::cout << str.str() << std::endl;
        filelist.push_back(str.str());
		image= cv::imread(str.str(),0);
        imshow("Image",image);
		 cv::waitKey(100);
	}
	// Create calibrator object
    CameraCalibrator cameraCalibrator;
	// add the corners from the chessboard
    cv::Size boardSize(8,6);
	cameraCalibrator.addChessboardPoints(
		filelist,	// filenames of chessboard image
        boardSize,
        24,24);	// size of chessboard

    cameraCalibrator.calibrate(image.size());   //相机校正




    for (int i=0; i<24; i++)
    {
        cv::Mat image2 = cv::imread(filelist[i]);
        imshow("Original Image", image2);

        cv::Mat uImage= cameraCalibrator.remap(image2);
        imshow("Undistorted Image", uImage);
         cv::waitKey(100);
    }


    cv::Mat cameraMatrix = cameraCalibrator.getCameraMatrix();
    cv::Mat DistCoeffsMatrix = cameraCalibrator.getDistCoeffs();
    std::vector<cv::Mat> rot_vectors = cameraCalibrator.getrot();
    std::vector<cv::Mat> trans_vectors = cameraCalibrator.gettrans();

// display camera matrix
    camcalibrate<< "Camera_Matrix" << cameraMatrix << "Distortion_Coefficients" << DistCoeffsMatrix;
    camcalibrate<< "rot_vectors" << rot_vectors << "trans_vectors" << trans_vectors;

    camcalibrate.release();//  xml文件完成

    FileStorage out("./data/camcalibrate_small_opencv.xml", FileStorage::READ);//  导出xml文件
    cv::Mat outMat =cv::Mat::eye(3, 3, CV_64F);
    cv::Mat outMat2 = cv::Mat::zeros(5, 1, CV_64F);

    out["Camera_Matrix"] >>  outMat ;
    out["Distortion_Coefficients"] >> outMat2 ;

    std::cout << " Camera intrinsic: " << outMat.rows << "x" << outMat.cols << std::endl;
    std::cout << outMat<< std::endl ;
    std::cout << " " << std::endl;
    std::cout << outMat.at<double>(0,0) << " " << outMat.at<double>(1,1) << " " << outMat.at<double>(2,2)<< " " << outMat.at<double>(0,2)<< " " << outMat.at<double>(1,2) << std::endl ;
    std::cout << " " << std::endl;
    std::cout << " DistCoeffsMatrix: " << outMat2.rows << "x" << outMat2.cols << std::endl;
    std::cout << outMat2<< std::endl ;

    cv::waitKey();
	return 0;

}

