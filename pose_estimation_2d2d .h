//
// Created by gintoki on 19-3-27.
//
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace cv;

#ifndef LIDAR_RECONSTRUCTION_CAMERA_VOID_POSE_ESTIMATION_2D2D_H
#define LIDAR_RECONSTRUCTION_CAMERA_VOID_POSE_ESTIMATION_2D2D_H
void pose_estimation_2d2d ( std::vector<KeyPoint> keypoints_1,
                            std::vector<KeyPoint> keypoints_2,
                            std::vector< DMatch > matches,
                            Mat& R, Mat& t );
#endif //LIDAR_RECONSTRUCTION_CAMERA_VOID_POSE_ESTIMATION_2D2D_H
