//
// Created by gintoki on 19-3-27.
//
#include <iostream>
using namespace std;
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv/cv.hpp>
#include "pose_estimation_2d2d .h"
void pose_estimation_2d2d ( std::vector<KeyPoint> keypoints_1,
                           std::vector<KeyPoint> keypoints_2,
                           std::vector< DMatch > matches,
                           Mat& R, Mat& t )
{
  // 相机内参,KITTI
  Mat K = ( Mat_<double> ( 3,3 ) << 9.842439e+02, 0, 6.900000e+02, 0, 9.808141e+02, 2.331966e+02, 0, 0, 1 );

  //-- 把匹配点转换为vector<Point2f>的形式
  vector<Point2f> points1;
  vector<Point2f> points2;

  for ( int i = 0; i < ( int ) matches.size(); i++ )
  {
    points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
    points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
  }

  //-- 计算本质矩阵
  Point2d principal_point ( 6.900000e+02, 2.331966e+02 );	//相机光心, KITTI dataset标定值
  double focal_length = 982;			//相机焦距, Kitti dataset标定值
  Mat essential_matrix;
  essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point ,RANSAC,0.99,1.0);
  cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

  //-- 从本质矩阵中恢复旋转和平移信息.
  recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
  cout<<"R is "<<endl<<R<<endl;
  cout<<"t is "<<endl<<t<<endl;

}